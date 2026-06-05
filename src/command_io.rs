//! Injectable command I/O adapter for conformance testing.
//!
//! This module provides a small abstraction over LR1110 command transport so
//! tests can verify full command/response exchanges without hardware.

use embedded_hal_async::spi::SpiDevice;
use lora_phy::lr1110::variant::Lr1110Variant;
use lora_phy::lr1110::Lr1110;
use lora_phy::mod_params::RadioError;
use lora_phy::mod_traits::InterfaceVariant;

/// Abstract command transport for LR1110 command/response exchanges.
pub trait CommandIo {
    /// Send a command that does not return payload bytes.
    async fn write_command(&mut self, cmd: &[u8]) -> Result<(), RadioError>;

    /// Send a command and read response payload bytes.
    async fn write_read_command(&mut self, cmd: &[u8], rsp: &mut [u8]) -> Result<(), RadioError>;
}

impl<SPI, IV, C> CommandIo for Lr1110<SPI, IV, C>
where
    SPI: SpiDevice<u8>,
    IV: InterfaceVariant,
    C: Lr1110Variant,
{
    async fn write_command(&mut self, cmd: &[u8]) -> Result<(), RadioError> {
        self.execute_command(cmd).await
    }

    async fn write_read_command(&mut self, cmd: &[u8], rsp: &mut [u8]) -> Result<(), RadioError> {
        self.execute_command_with_response(cmd, rsp).await
    }
}

/// Small helper that builds and executes command frames over an injected transport.
pub struct CommandRunner<IO> {
    io: IO,
}

impl<IO> CommandRunner<IO>
where
    IO: CommandIo,
{
    /// Create a command runner with any `CommandIo` implementation.
    pub fn new(io: IO) -> Self {
        Self { io }
    }

    /// Consume the runner and return the underlying transport.
    pub fn into_inner(self) -> IO {
        self.io
    }

    /// Manual command `GetVbat` (`0x0119`).
    pub async fn system_get_vbat(&mut self) -> Result<u8, RadioError> {
        let cmd = [0x01, 0x19];
        let mut rsp = [0u8; 1];
        self.io.write_read_command(&cmd, &mut rsp).await?;
        Ok(rsp[0])
    }

    /// Manual command `WifiGetNbResults` (`0x0305`).
    pub async fn wifi_get_nb_results(&mut self) -> Result<u8, RadioError> {
        let cmd = [0x03, 0x05];
        let mut rsp = [0u8; 1];
        self.io.write_read_command(&cmd, &mut rsp).await?;
        Ok(rsp[0])
    }

    /// Manual command `GnssGetResultSize` (`0x040C`).
    pub async fn gnss_get_result_size(&mut self) -> Result<u16, RadioError> {
        let cmd = [0x04, 0x0C];
        let mut rsp = [0u8; 2];
        self.io.write_read_command(&cmd, &mut rsp).await?;
        Ok(((rsp[0] as u16) << 8) | (rsp[1] as u16))
    }

    /// Manual command `SetTcxoMode` (`0x0117`).
    ///
    /// `voltage` is the raw tune value from the user manual (0x00..0x07).
    /// `timeout` is encoded on 24 bits in RTC steps.
    pub async fn system_set_tcxo_mode(&mut self, voltage: u8, timeout: u32) -> Result<(), RadioError> {
        if voltage > 0x07 {
            return Err(RadioError::InvalidConfiguration);
        }
        if timeout > 0x00FF_FFFF {
            return Err(RadioError::PayloadSizeUnexpected(timeout as usize));
        }
        let cmd = [
            0x01,
            0x17,
            voltage,
            ((timeout >> 16) & 0xFF) as u8,
            ((timeout >> 8) & 0xFF) as u8,
            (timeout & 0xFF) as u8,
        ];
        self.io.write_command(&cmd).await
    }

    /// Manual command `WifiScan` (`0x0300`).
    pub async fn wifi_scan(
        &mut self,
        wifi_type: u8,
        channel_mask: u16,
        scan_mode: u8,
        max_results: u8,
        nb_scan_per_channel: u8,
        timeout: u16,
        abort_on_timeout: bool,
    ) -> Result<(), RadioError> {
        if max_results == 0 {
            return Err(RadioError::PayloadSizeUnexpected(max_results as usize));
        }
        if nb_scan_per_channel == 0 {
            return Err(RadioError::PayloadSizeUnexpected(nb_scan_per_channel as usize));
        }
        if timeout == 0 {
            return Err(RadioError::PayloadSizeUnexpected(timeout as usize));
        }
        // Channel mask bits above 14 channels are invalid.
        if channel_mask & 0xC000 != 0 {
            return Err(RadioError::InvalidConfiguration);
        }
        let cmd = [
            0x03,
            0x00,
            wifi_type,
            (channel_mask >> 8) as u8,
            channel_mask as u8,
            scan_mode,
            max_results,
            nb_scan_per_channel,
            (timeout >> 8) as u8,
            timeout as u8,
            abort_on_timeout as u8,
        ];
        self.io.write_command(&cmd).await
    }

    /// Manual unified-scan command used by this crate (`0x040B`).
    pub async fn gnss_scan(
        &mut self,
        effort_mode: u8,
        result_mask: u8,
        nb_sv_max: u8,
    ) -> Result<(), RadioError> {
        if effort_mode > 0x02 {
            return Err(RadioError::InvalidConfiguration);
        }
        let cmd = [0x04, 0x0B, effort_mode, result_mask, nb_sv_max];
        self.io.write_command(&cmd).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::future::Future;
    use futures::executor::block_on;
    extern crate std;
    use std::collections::VecDeque;
    use std::vec::Vec;

    #[derive(Default)]
    struct MockCommandIo {
        writes: Vec<Vec<u8>>,
        responses: VecDeque<Vec<u8>>,
    }

    impl MockCommandIo {
        fn push_response(&mut self, rsp: &[u8]) {
            self.responses.push_back(rsp.to_vec());
        }
    }

    impl CommandIo for MockCommandIo {
        async fn write_command(&mut self, cmd: &[u8]) -> Result<(), RadioError> {
            self.writes.push(cmd.to_vec());
            Ok(())
        }

        async fn write_read_command(
            &mut self,
            cmd: &[u8],
            rsp: &mut [u8],
        ) -> Result<(), RadioError> {
            self.writes.push(cmd.to_vec());
            let next = self
                .responses
                .pop_front()
                .ok_or(RadioError::PayloadSizeMismatch(0, rsp.len()))?;
            if next.len() != rsp.len() {
                return Err(RadioError::PayloadSizeMismatch(next.len(), rsp.len()));
            }
            rsp.copy_from_slice(&next);
            Ok(())
        }
    }

    fn run<F>(f: F)
    where
        F: Future<Output = ()>,
    {
        block_on(f);
    }

    #[test]
    fn runner_system_get_vbat_matches_manual_exchange() {
        run(async {
            let mut io = MockCommandIo::default();
            io.push_response(&[0x8F]);

            let mut runner = CommandRunner::new(io);
            let vbat = runner.system_get_vbat().await.expect("get_vbat failed");
            let io = runner.into_inner();

            assert_eq!(vbat, 0x8F);
            assert_eq!(io.writes.len(), 1);
            assert_eq!(io.writes[0], [0x01, 0x19].to_vec());
        });
    }

    #[test]
    fn runner_wifi_get_nb_results_matches_manual_exchange() {
        run(async {
            let mut io = MockCommandIo::default();
            io.push_response(&[7]);

            let mut runner = CommandRunner::new(io);
            let count = runner
                .wifi_get_nb_results()
                .await
                .expect("wifi_get_nb_results failed");
            let io = runner.into_inner();

            assert_eq!(count, 7);
            assert_eq!(io.writes.len(), 1);
            assert_eq!(io.writes[0], [0x03, 0x05].to_vec());
        });
    }

    #[test]
    fn runner_gnss_get_result_size_matches_manual_exchange() {
        run(async {
            let mut io = MockCommandIo::default();
            io.push_response(&[0x01, 0x2C]); // 300 bytes

            let mut runner = CommandRunner::new(io);
            let size = runner
                .gnss_get_result_size()
                .await
                .expect("gnss_get_result_size failed");
            let io = runner.into_inner();

            assert_eq!(size, 300);
            assert_eq!(io.writes.len(), 1);
            assert_eq!(io.writes[0], [0x04, 0x0C].to_vec());
        });
    }

    #[test]
    fn runner_system_set_tcxo_mode_matches_manual_exchange() {
        run(async {
            let io = MockCommandIo::default();
            let mut runner = CommandRunner::new(io);

            // Voltage 2.7V is 0x05 in LR1110 UM tables.
            runner
                .system_set_tcxo_mode(0x05, 0x00A1B2)
                .await
                .expect("system_set_tcxo_mode failed");
            let io = runner.into_inner();

            assert_eq!(io.writes.len(), 1);
            assert_eq!(io.writes[0], [0x01, 0x17, 0x05, 0x00, 0xA1, 0xB2].to_vec());
        });
    }

    #[test]
    fn runner_wifi_scan_matches_manual_exchange() {
        run(async {
            let io = MockCommandIo::default();
            let mut runner = CommandRunner::new(io);

            // Equivalent to channels 1, 6, 11 mask (0x0421).
            runner
                .wifi_scan(0x01, 0x0421, 0x00, 16, 1, 100, false)
                .await
                .expect("wifi_scan failed");
            let io = runner.into_inner();

            assert_eq!(io.writes.len(), 1);
            assert_eq!(
                io.writes[0],
                [0x03, 0x00, 0x01, 0x04, 0x21, 0x00, 16, 1, 0x00, 0x64, 0x00].to_vec()
            );
        });
    }

    #[test]
    fn runner_gnss_scan_matches_manual_exchange() {
        run(async {
            let io = MockCommandIo::default();
            let mut runner = CommandRunner::new(io);

            // High effort (0x02), all result fields (0x07), no explicit SV max (0).
            runner
                .gnss_scan(0x02, 0x07, 0x00)
                .await
                .expect("gnss_scan failed");
            let io = runner.into_inner();

            assert_eq!(io.writes.len(), 1);
            assert_eq!(io.writes[0], [0x04, 0x0B, 0x02, 0x07, 0x00].to_vec());
        });
    }

    #[test]
    fn runner_fails_when_response_queue_empty() {
        run(async {
            let io = MockCommandIo::default();
            let mut runner = CommandRunner::new(io);

            let err = runner
                .system_get_vbat()
                .await
                .expect_err("expected response-queue-empty error");

            assert_eq!(err, RadioError::PayloadSizeMismatch(0, 1));
        });
    }

    #[test]
    fn runner_fails_on_response_length_mismatch() {
        run(async {
            let mut io = MockCommandIo::default();
            io.push_response(&[0xAA]); // gnss_get_result_size expects 2 bytes
            let mut runner = CommandRunner::new(io);

            let err = runner
                .gnss_get_result_size()
                .await
                .expect_err("expected response-length mismatch error");

            assert_eq!(err, RadioError::PayloadSizeMismatch(1, 2));
        });
    }

    #[test]
    fn runner_rejects_invalid_tcxo_voltage() {
        run(async {
            let io = MockCommandIo::default();
            let mut runner = CommandRunner::new(io);

            let err = runner
                .system_set_tcxo_mode(0x08, 100)
                .await
                .expect_err("expected invalid tcxo voltage error");

            assert_eq!(err, RadioError::InvalidConfiguration);
        });
    }

    #[test]
    fn runner_rejects_invalid_wifi_scan_inputs() {
        run(async {
            let io = MockCommandIo::default();
            let mut runner = CommandRunner::new(io);

            let err = runner
                .wifi_scan(0x01, 0xFFFF, 0x00, 16, 1, 100, false)
                .await
                .expect_err("expected invalid channel mask error");
            assert_eq!(err, RadioError::InvalidConfiguration);
        });
    }

    #[test]
    fn runner_rejects_invalid_gnss_effort_mode() {
        run(async {
            let io = MockCommandIo::default();
            let mut runner = CommandRunner::new(io);

            let err = runner
                .gnss_scan(0x03, 0x07, 0x00)
                .await
                .expect_err("expected invalid gnss effort mode error");

            assert_eq!(err, RadioError::InvalidConfiguration);
        });
    }
}
