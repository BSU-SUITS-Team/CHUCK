import { readable } from 'svelte/store';

export type Range = {
	units: "(seconds)" | "(%)" | "(psi)" | "(bpm)" | "(psi/min)" | "(rpm)" | "(℉)",
	min: number,
	nominal: number | undefined,
	max: number;
};

export enum Threshold {
	Min = -1,
	Nominal = 0,
	Max = 1,
};

export type Category = "Resources" | "Atmosphere" | "Helmet" | "Scrubber" | "Temperature";

export type Resources = {
	batt_time_left: number,
	oxy_pri_storage: number,
	oxy_sec_storage: number,
	oxy_pri_pressure: number,
	oxy_sec_pressure: number,
	oxy_time_left: number,
	coolant_ml: number,
};

export type Atmosphere = {
	heart_rate: number,
	oxy_consumption: number,
	co2_production: number,
	suit_pressure_oxy: number,
	suit_pressure_co2: number,
	suit_pressure_other: number,
	suit_pressure_total: number,
	helmet_pressure_co2: number,
};

export type Helmet = {
	fan_pri_rpm: number,
	fan_sec_rpm: number,
};

export type Scrubbers = {
	scrubber_a_co2_storage: number,
	scrubber_b_co2_storage: number,
};

export type Temperature = {
	temperature: number,
	coolant_gas_pressure: number,
	coolant_liquid_pressure: number,
}

export type Astronaut =
	Resources &
	Atmosphere &
	Helmet &
	Scrubbers &
	Temperature;

export type Telemetry = {
	eva_time: number,
	[eva: string]: Astronaut | number,
};

export type TelemetryEvent = {
	telemetry: Telemetry,
};

export type Bounds<T> = { [K in keyof T]: Range }

export const ResourceBounds: Bounds<Resources> = {
	batt_time_left: {
		units: '(seconds)',
		min: 3600,
		nominal: undefined,
		max: 10800,
	},
	oxy_pri_storage: {
		units: '(%)',
		min: 20,
		nominal: undefined,
		max: 100,
	},
	oxy_sec_storage: {
		units: '(%)',
		min: 20,
		nominal: undefined,
		max: 100,
	},
	oxy_pri_pressure: {
		units: '(psi)',
		min: 600,
		nominal: undefined,
		max: 3000,
	},
	oxy_sec_pressure: {
		units: '(psi)',
		min: 600,
		nominal: undefined,
		max: 3000,
	},
	oxy_time_left: {
		units: '(seconds)',
		min: 3600,
		nominal: undefined,
		max: 21000
	},
	coolant_ml: {
		units: '(%)',
		min: 80,
		nominal: 100,
		max: 100,
	}
};

export const AtmosphereBounds: Bounds<Atmosphere> = {
	heart_rate: {
		units: '(bpm)',
		min: 50,
		nominal: 90,
		max: 160,
	},
	oxy_consumption: {
		units: '(psi/min)',
		min: 0.05,
		nominal: 0.1,
		max: 0.15,
	},
	co2_production: {
		units: '(psi/min)',
		min: 0.05,
		nominal: 0.1,
		max: 0.15,
	},
	suit_pressure_oxy: {
		units: '(psi)',
		min: 3.5,
		nominal: 4.0,
		max: 4.1,
	},
	suit_pressure_co2: {
		units: '(psi)',
		min: 0.0,
		nominal: 0.0,
		max: 0.1,
	},
	suit_pressure_other: {
		units: '(psi)',
		min: 0.0,
		nominal: 0.0,
		max: 0.5,
	},
	suit_pressure_total: {
		units: '(psi)',
		min: 3.5,
		nominal: 4.0,
		max: 4.5,
	},
	helmet_pressure_co2: {
		units: '(seconds)',
		min: 0.0,
		nominal: 0.1,
		max: 0.15,
	}
};

export const HelmetBounds: Bounds<Helmet> = {
	fan_pri_rpm: {
		units: '(rpm)',
		min: 20_000,
		nominal: 30_000,
		max: 30_000,
	},
	fan_sec_rpm: {
		units: '(rpm)',
		min: 20_000,
		nominal: 30_000,
		max: 30_000,
	}
};

export const ScrubberBounds: Bounds<Scrubbers> = {
	scrubber_a_co2_storage: {
		units: '(%)',
		min: 0,
		nominal: undefined,
		max: 60,
	},
	scrubber_b_co2_storage: {
		units: '(%)',
		min: 0,
		nominal: undefined,
		max: 60,
	}
};

export const TemperatureBounds: Bounds<Temperature> = {
	temperature: {
		units: '(℉)',
		min: 50,
		nominal: 70,
		max: 90,
	},
	coolant_gas_pressure: {
		units: '(psi)',
		min: 100,
		nominal: 500,
		max: 700,
	},
	coolant_liquid_pressure: {
		units: '(psi)',
		min: 0,
		nominal: 0,
		max: 700,
	}
};

export const sampleTelemetry: TelemetryEvent = {
	"telemetry": {
		"eva_time": 0,
		"eva1": {
			"batt_time_left": 5077.148926,
			"oxy_pri_storage": 23.755802,
			"oxy_sec_storage": 15.489529,
			"oxy_pri_pressure": 0.000000,
			"oxy_sec_pressure": 0.000000,
			"oxy_time_left": 4238,
			"heart_rate": 90.000000,
			"oxy_consumption": 0.000000,
			"co2_production": 0.000000,
			"suit_pressure_oxy": 3.072300,
			"suit_pressure_co2": 0.005900,
			"suit_pressure_other": 11.554200,
			"suit_pressure_total": 14.632401,
			"fan_pri_rpm": 0.000000,
			"fan_sec_rpm": 0.000000,
			"helmet_pressure_co2": 0.000000,
			"scrubber_a_co2_storage": 0.000000,
			"scrubber_b_co2_storage": 0.000000,
			"temperature": 70.000000,
			"coolant_ml": 20.508068,
			"coolant_gas_pressure": 0.000000,
			"coolant_liquid_pressure": 0.000000
		},
		"eva2": {
			"batt_time_left": 3384.893799,
			"oxy_pri_storage": 24.231962,
			"oxy_sec_storage": 19.419136,
			"oxy_pri_pressure": 0.000000,
			"oxy_sec_pressure": 0.000000,
			"oxy_time_left": 4714,
			"heart_rate": 90.000000,
			"oxy_consumption": 0.000000,
			"co2_production": 0.000000,
			"suit_pressure_oxy": 3.072300,
			"suit_pressure_co2": 0.005900,
			"suit_pressure_other": 11.554200,
			"suit_pressure_total": 14.632401,
			"fan_pri_rpm": 0.000000,
			"fan_sec_rpm": 0.000000,
			"helmet_pressure_co2": 0.000000,
			"scrubber_a_co2_storage": 0.000000,
			"scrubber_b_co2_storage": 0.000000,
			"temperature": 70.000000,
			"coolant_ml": 22.034748,
			"coolant_gas_pressure": 0.000000,
			"coolant_liquid_pressure": 0.000000
		}
	}
};

export function getAstronauts(event: TelemetryEvent): string[] {
	const { telemetry } = event;
	const keys = Object.keys(telemetry).filter((key, _) => key != "eva_time");
	return keys;
}

export function getEVA(event: TelemetryEvent, eva: string): Astronaut | undefined {
	if (eva !== "eva_time" && !getAstronauts(event).includes(eva)) {
		console.log(`Unable to find astronaut: ${eva} in telemetry event!`, event, eva);
		return undefined;
	}
	const { telemetry } = event;
	return telemetry[eva] as Astronaut;
}

export function compareValueToBounds(number: number, range: Range): Threshold {
	if (number <= range.min) return Threshold.Min;
	if (number < range.max) return Threshold.Nominal;
	return Threshold.Max;
}