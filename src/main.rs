use std::{
    time::{
        Instant,
        Duration,
    },
};

use structopt::{
    clap::{
        arg_enum,
        AppSettings,
    },
    StructOpt,
};

use rpi_lfa::{
    Volt,
    Hertz,
    ac_driver,
};

use rpi_lfa_rppal::{
    mcp3008,
};

#[derive(Debug)]
enum Error {
    ChannelIsNotInRangeFrom0To7 { provided: usize, },
    Mcp3008(mcp3008::Error),
}

#[derive(Clone, StructOpt, Debug)]
#[structopt(setting = AppSettings::DeriveDisplayOrder)]
struct CliArgs {
    #[structopt(flatten)]
    pub mcp3008: CliMcp3008,
    /// dump stats delay timeout (in milliseconds)
    #[structopt(long = "dump-stats-delay", short = "t", default_value = "1000")]
    dump_stats_delay: usize,
}

#[derive(Clone, StructOpt, Debug)]
#[structopt(setting = AppSettings::DeriveDisplayOrder)]
struct CliMcp3008 {
    /// mcp3008 carrier channel
    #[structopt(long = "mcp3008-carrier-channel", short = "c", default_value = "0")]
    carrier_channel: usize,
    /// mcp3008 voltage drain (vdd) in volts
    #[structopt(long = "mcp3008-voltage-drain", short = "v", possible_values = &CliMcp3008Vdd::variants(), case_insensitive = true)]
    voltage_drain: CliMcp3008Vdd,
    /// mcp3008 voltage reference in volts
    #[structopt(long = "mcp3008-voltage-reference", short = "r")]
    voltage_ref: Option<f64>,
}

arg_enum! {
    #[derive(Clone, Debug)]
    enum CliMcp3008Vdd {
        Positive3v3,
        Positive5v,
    }
}

fn main() -> Result<(), Error> {
    pretty_env_logger::init_timed();
    let cli_args = CliArgs::from_args();
    log::info!("program started as: {:?}", cli_args);

    let mcp3008_channel = match cli_args.mcp3008.carrier_channel {
        0 => mcp3008::Channel::Ch0,
        1 => mcp3008::Channel::Ch1,
        2 => mcp3008::Channel::Ch2,
        3 => mcp3008::Channel::Ch3,
        4 => mcp3008::Channel::Ch4,
        5 => mcp3008::Channel::Ch5,
        6 => mcp3008::Channel::Ch6,
        7 => mcp3008::Channel::Ch7,
        provided => return Err(Error::ChannelIsNotInRangeFrom0To7 { provided, }),
    };

    let mcp3008_params = mcp3008::Params {
        voltage_drain: match cli_args.mcp3008.voltage_drain {
            CliMcp3008Vdd::Positive3v3 =>
                mcp3008::Vdd::Positive3v3,
            CliMcp3008Vdd::Positive5v =>
                mcp3008::Vdd::Positive5v,
        },
        voltage_ref: match cli_args.mcp3008.voltage_ref {
            None =>
                mcp3008::Vref::EqualToVdd,
            Some(value) =>
                mcp3008::Vref::Other {
                    voltage: Volt(value),
                },
        },
    };

    let dump_delay = Duration::from_millis(cli_args.dump_stats_delay as u64);

    let mut ac_last_dump = Instant::now();
    let mut ac_samples = 0;
    let mut ac_avg_hz = 0.0;
    let mut ac_avg_hi = 0.0;
    let mut ac_avg_lo = 0.0;

    let mut ac_driver_session = ac_driver::Session::new();
    let mut mcp3008_session = mcp3008::Session::new(&mcp3008_params)
        .map_err(Error::Mcp3008)?;

    loop {
        let mut channel_voltage_read = None;

        match mcp3008_session {

            mcp3008::Session::Initializing(initializing) =>
                match initializing.probe().map_err(Error::Mcp3008)? {
                    mcp3008::InitializingOp::Idle(initializing) =>
                        mcp3008_session = initializing.into(),
                    mcp3008::InitializingOp::Ready(ready) => {
                        log::debug!("mcp3008 ready");
                        mcp3008_session = ready.into();
                    },
                },

            mcp3008::Session::Ready(ready) =>
                mcp3008_session = ready.probe_channel(mcp3008_channel).into(),

            mcp3008::Session::Probing(probing) =>
                match probing.poll().map_err(Error::Mcp3008)? {
                    mcp3008::ProbingOp::Idle(probing) =>
                        mcp3008_session = probing.into(),
                    mcp3008::ProbingOp::Done { channel, value, ready, } if channel == mcp3008_channel => {
                        channel_voltage_read = Some(value);
                        mcp3008_session = ready.into();
                    },
                    mcp3008::ProbingOp::Done { ready, .. } =>
                        mcp3008_session = ready.into(),
                },

        }

        if let Some(voltage) = channel_voltage_read {
            let mut ac_values_read = None;
            let now = Instant::now();
            match ac_driver_session {

                ac_driver::Session::Initializing(initializing) =>
                    match initializing.voltage_read(now, voltage) {
                        ac_driver::InitializingOp::Idle(initializing) =>
                            ac_driver_session = initializing.into(),
                        ac_driver::InitializingOp::CarrierDetected(estimated) => {
                            log::debug!("ac_driver carrier detected");
                            ac_values_read = Some(estimated.values().clone());
                            ac_driver_session = estimated.into();
                        },
                    },

                ac_driver::Session::Estimated(estimated) =>
                    match estimated.voltage_read(now, voltage) {
                        ac_driver::EstimatedOp::Idle(estimated) => {
                            ac_values_read = Some(estimated.values().clone());
                            ac_driver_session = estimated.into();
                        },
                        ac_driver::EstimatedOp::CarrierLost(initializing) => {
                            log::debug!("ac_driver carrier lost");
                            ac_driver_session = initializing.into();
                        },
                    },

            }

            if let Some(ac_values) = ac_values_read {
                ac_samples += 1;
                ac_avg_hz += ac_values.frequency.0;
                ac_avg_hi += ac_values.amplitude.max.value.0;
                ac_avg_lo += ac_values.amplitude.min.value.0;
            }

            if now.duration_since(ac_last_dump) >= dump_delay {
                if ac_samples == 0 {
                    log::info!("no samples collected yet");
                } else {
                    log::info!(" - avg frequency: {:?}", Hertz(ac_avg_hz / ac_samples as f64));
                    log::info!(" - avg amplitude hi: {:?}", Volt(ac_avg_hi / ac_samples as f64));
                    log::info!(" - avg amplitude lo: {:?}", Volt(ac_avg_lo / ac_samples as f64));
                }
                ac_samples = 0;
                ac_avg_hz = 0.0;
                ac_avg_hi = 0.0;
                ac_avg_lo = 0.0;
                ac_last_dump = now;
            }
        }
    }
}
