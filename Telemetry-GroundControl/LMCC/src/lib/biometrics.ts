export type Range = {
	units: '' | '%' | 'psi' | 'bpm' | 'psi/min' | 'rpm' | 'C';
	min: number;
	nominal: number | undefined;
	max: number;
	limit: [number, number];
};

export enum Threshold {
	Min = -1,
	Nominal = 0,
	Max = 1
}

export type Category = 'Resources' | 'Atmosphere' | 'Helmet' | 'Scrubber' | 'Temperature';

export type Resources = {
	primary_battery_level?: number;
	secondary_battery_level?: number;
	battery_level?: number;
	oxy_pri_storage: number;
	oxy_sec_storage: number;
	oxy_pri_pressure: number;
	oxy_sec_pressure: number;
	coolant_storage: number;
	eva_elapsed_time: number;
};

export type Atmosphere = {
	heart_rate: number;
	oxy_consumption: number;
	co2_production: number;
	suit_pressure_oxy: number;
	suit_pressure_co2: number;
	suit_pressure_other: number;
	suit_pressure_total: number;
	helmet_pressure_co2: number;
};

export type Helmet = {
	fan_pri_rpm: number;
	fan_sec_rpm: number;
};

export type Scrubbers = {
	scrubber_a_co2_storage: number;
	scrubber_b_co2_storage: number;
};

export type Temperature = {
	temperature: number;
	coolant_gas_pressure: number;
	coolant_liquid_pressure: number;
};

export type Astronaut = Resources & Atmosphere & Helmet & Scrubbers & Temperature;

export type TelemetryEvent = {
	time?: number;
	[eva: string]: Astronaut | number | undefined;
};

export type EVAPayload = {
	telemetry: TelemetryEvent;
	status?: Record<string, unknown>;
	dcu?: Record<string, unknown>;
	error?: Record<string, unknown>;
	imu?: Record<string, unknown>;
	uia?: Record<string, unknown>;
};

export type Telemetry = TelemetryEvent | EVAPayload;

export type Bounds<T> = { [K in keyof T]-?: Range };

export const ResourceBounds: Bounds<Resources> = {
	primary_battery_level: {
		units: '%',
		min: 20,
		nominal: undefined,
		max: 100,
		limit: [0, 100]
	},
	secondary_battery_level: {
		units: '%',
		min: 20,
		nominal: undefined,
		max: 100,
		limit: [0, 100]
	},
	battery_level: {
		units: '%',
		min: 20,
		nominal: undefined,
		max: 100,
		limit: [0, 100]
	},
	oxy_pri_storage: {
		units: '%',
		min: 20,
		nominal: undefined,
		max: 100,
		limit: [0, 100]
	},
	oxy_sec_storage: {
		units: '%',
		min: 20,
		nominal: undefined,
		max: 100,
		limit: [0, 100]
	},
	oxy_pri_pressure: {
		units: 'psi',
		min: 600,
		nominal: undefined,
		max: 3000,
		limit: [0, 4000]
	},
	oxy_sec_pressure: {
		units: 'psi',
		min: 600,
		nominal: undefined,
		max: 3000,
		limit: [0, 4000]
	},
	coolant_storage: {
		units: '%',
		min: 20,
		nominal: undefined,
		max: 100,
		limit: [0, 100]
	},
	eva_elapsed_time: {
		units: '',
		min: 0,
		nominal: undefined,
		max: 36000,
		limit: [0, 36000]
	}
};

export const AtmosphereBounds: Bounds<Atmosphere> = {
	heart_rate: {
		units: 'bpm',
		min: 50,
		nominal: 90,
		max: 160,
		limit: [0, 250]
	},
	oxy_consumption: {
		units: 'psi/min',
		min: 0.05,
		nominal: 0.1,
		max: 0.15,
		limit: [0, 0.2]
	},
	co2_production: {
		units: 'psi/min',
		min: 0.05,
		nominal: 0.1,
		max: 0.15,
		limit: [0, 0.2]
	},
	suit_pressure_oxy: {
		units: 'psi',
		min: 3.5,
		nominal: 4.0,
		max: 4.1,
		limit: [0, 4.5]
	},
	suit_pressure_co2: {
		units: 'psi',
		min: 0.0,
		nominal: 0.0,
		max: 0.1,
		limit: [0, 0.15]
	},
	suit_pressure_other: {
		units: 'psi',
		min: 0.0,
		nominal: 0.0,
		max: 0.5,
		limit: [0, 0.5]
	},
	suit_pressure_total: {
		units: 'psi',
		min: 3.5,
		nominal: 4.0,
		max: 4.5,
		limit: [0, 5]
	},
	helmet_pressure_co2: {
		units: 'psi',
		min: 0.0,
		nominal: 0.1,
		max: 0.15,
		limit: [0, 0.2]
	}
};

export const HelmetBounds: Bounds<Helmet> = {
	fan_pri_rpm: {
		units: 'rpm',
		min: 20_000,
		nominal: 30_000,
		max: 30_000,
		limit: [0, 40000]
	},
	fan_sec_rpm: {
		units: 'rpm',
		min: 20_000,
		nominal: 30_000,
		max: 30_000,
		limit: [0, 40000]
	}
};

export const ScrubberBounds: Bounds<Scrubbers> = {
	scrubber_a_co2_storage: {
		units: '%',
		min: 0,
		nominal: undefined,
		max: 60,
		limit: [0, 100]
	},
	scrubber_b_co2_storage: {
		units: '%',
		min: 0,
		nominal: undefined,
		max: 60,
		limit: [0, 100]
	}
};

export const TemperatureBounds: Bounds<Temperature> = {
	temperature: {
		units: 'C',
		min: 10,
		nominal: 21,
		max: 32,
		limit: [0, 45]
	},
	coolant_gas_pressure: {
		units: 'psi',
		min: 100,
		nominal: 500,
		max: 700,
		limit: [0, 800]
	},
	coolant_liquid_pressure: {
		units: 'psi',
		min: 0,
		nominal: 500,
		max: 700,
		limit: [0, 800]
	}
};

export const sampleTelemetry: EVAPayload = {
	telemetry: {
		eva1: {
			primary_battery_level: 100,
			secondary_battery_level: 100,
			oxy_pri_storage: 100,
			oxy_sec_storage: 100,
			oxy_pri_pressure: 0,
			oxy_sec_pressure: 0,
			suit_pressure_oxy: 4,
			suit_pressure_co2: 0,
			suit_pressure_other: 0,
			suit_pressure_total: 0,
			helmet_pressure_co2: 0,
			fan_pri_rpm: 0,
			fan_sec_rpm: 30000,
			scrubber_a_co2_storage: 0,
			scrubber_b_co2_storage: 0,
			temperature: 21.100000381469727,
			coolant_storage: 100,
			coolant_gas_pressure: 0,
			coolant_liquid_pressure: 500,
			heart_rate: 0,
			oxy_consumption: 0,
			co2_production: 0,
			eva_elapsed_time: 0
		},
		eva2: {
			battery_level: 100,
			oxy_pri_storage: 100,
			oxy_sec_storage: 100,
			oxy_pri_pressure: 0,
			oxy_sec_pressure: 0,
			suit_pressure_oxy: 4,
			suit_pressure_co2: 0.05000000074505806,
			suit_pressure_other: 0,
			suit_pressure_total: 0,
			helmet_pressure_co2: 0,
			fan_pri_rpm: 0,
			fan_sec_rpm: 0,
			scrubber_a_co2_storage: 30,
			scrubber_b_co2_storage: 30,
			temperature: 21.100000381469727,
			coolant_storage: 0,
			coolant_gas_pressure: 0,
			coolant_liquid_pressure: 500,
			heart_rate: 0,
			oxy_consumption: 0,
			co2_production: 0,
			eva_elapsed_time: 0
		}
	},
	status: {
		started: false
	},
	dcu: {
		eva1: {
			oxy: false,
			fan: false,
			pump: false,
			co2: false,
			batt: {
				lu: false,
				ps: false
			}
		},
		eva2: {
			batt: false,
			oxy: false,
			comm: false,
			fan: false,
			pump: false,
			co2: false
		}
	},
	error: {
		fan_error: false,
		oxy_error: false,
		power_error: false,
		scrubber_error: false
	},
	imu: {
		eva1: {
			posx: -6804.291504,
			posy: -10868.504883,
			heading: 0
		},
		eva2: {
			posx: -6804.300781,
			posy: -10868.552734,
			heading: 0
		}
	},
	uia: {
		eva1_power: false,
		eva1_oxy: false,
		eva1_water_supply: false,
		eva1_water_waste: false,
		eva2_power: false,
		eva2_oxy: false,
		eva2_water_supply: false,
		eva2_water_waste: false,
		oxy_vent: false,
		depress: false
	}
};

export function getTelemetryEvent(event: Telemetry | undefined): TelemetryEvent {
	if (!event) return {};

	if ('telemetry' in event && event.telemetry) {
		return event.telemetry;
	}

	return event as TelemetryEvent;
}

export function getAstronauts(event: Telemetry | undefined): string[] {
	const telemetry = getTelemetryEvent(event);
	return Object.keys(telemetry).filter((key) => {
		const value = telemetry[key];
		return key !== 'time' && typeof value === 'object' && value !== null;
	});
}

export function getEVA(event: Telemetry | undefined, eva: string): Astronaut | undefined {
	const telemetry = getTelemetryEvent(event);
	if (!getAstronauts(event).includes(eva)) {
		return undefined;
	}

	return telemetry[eva] as Astronaut;
}

export function compareValueToBounds(number: number, range: Range): Threshold {
	if (number <= range.min) return Threshold.Min;
	if (number < range.max) return Threshold.Nominal;
	return Threshold.Max;
}
