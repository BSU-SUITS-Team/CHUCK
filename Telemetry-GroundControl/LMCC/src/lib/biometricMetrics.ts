import {
	AtmosphereBounds,
	HelmetBounds,
	ResourceBounds,
	ScrubberBounds,
	TemperatureBounds,
	type Astronaut,
	type Range
} from './biometrics';
import { formatDecimals, formatTime } from './formatting';

export type Metric = {
	key: string;
	shortKey: string;
	units: Range['units'];
	formatter: (value: number) => string | number;
	range: Range;
	value: number;
};

export type FlatMetric = Metric & {
	category: string;
	categoryCode: string;
};

export type MetricCategoryMap = Record<string, Metric[]>;
export type MetricStatus = 'low' | 'nominal' | 'high';

const twoDecimals = formatDecimals(2);

const categoryCodes: Record<string, string> = {
	'Suit Resources': 'RES',
	'Suit Atmosphere': 'ATM',
	'Suit Helmet Fan': 'FAN',
	'Suit CO2 Scrubbers': 'SCR',
	'Suit Temperature': 'TMP'
};

const suitResources = (eva: Astronaut): Metric[] => {
	const resources: Metric[] = [
		{
			key: 'EVA Elapsed Time',
			shortKey: 'EVA Time',
			units: ResourceBounds.eva_elapsed_time.units,
			formatter: formatTime,
			range: ResourceBounds.eva_elapsed_time,
			value: eva.eva_elapsed_time
		}
	];

	if (typeof eva.primary_battery_level === 'number') {
		resources.push({
			key: 'Primary Battery Level',
			shortKey: 'Pri Batt',
			units: ResourceBounds.primary_battery_level.units,
			formatter: twoDecimals,
			range: ResourceBounds.primary_battery_level,
			value: eva.primary_battery_level
		});
	}

	if (typeof eva.secondary_battery_level === 'number') {
		resources.push({
			key: 'Secondary Battery Level',
			shortKey: 'Sec Batt',
			units: ResourceBounds.secondary_battery_level.units,
			formatter: twoDecimals,
			range: ResourceBounds.secondary_battery_level,
			value: eva.secondary_battery_level
		});
	}

	if (typeof eva.battery_level === 'number') {
		resources.push({
			key: 'Battery Level',
			shortKey: 'Battery',
			units: ResourceBounds.battery_level.units,
			formatter: twoDecimals,
			range: ResourceBounds.battery_level,
			value: eva.battery_level
		});
	}

	return [
		...resources,
		{
			key: 'Primary Oxygen Storage',
			shortKey: 'Pri O2 Store',
			units: ResourceBounds.oxy_pri_storage.units,
			formatter: twoDecimals,
			range: ResourceBounds.oxy_pri_storage,
			value: eva.oxy_pri_storage
		},
		{
			key: 'Secondary Oxygen Storage',
			shortKey: 'Sec O2 Store',
			units: ResourceBounds.oxy_sec_storage.units,
			formatter: twoDecimals,
			range: ResourceBounds.oxy_sec_storage,
			value: eva.oxy_sec_storage
		},
		{
			key: 'Primary Oxygen Pressure',
			shortKey: 'Pri O2 PSI',
			units: ResourceBounds.oxy_pri_pressure.units,
			formatter: twoDecimals,
			range: ResourceBounds.oxy_pri_pressure,
			value: eva.oxy_pri_pressure
		},
		{
			key: 'Secondary Oxygen Pressure',
			shortKey: 'Sec O2 PSI',
			units: ResourceBounds.oxy_sec_pressure.units,
			formatter: twoDecimals,
			range: ResourceBounds.oxy_sec_pressure,
			value: eva.oxy_sec_pressure
		},
		{
			key: 'Coolant Storage',
			shortKey: 'Coolant',
			units: ResourceBounds.coolant_storage.units,
			formatter: twoDecimals,
			range: ResourceBounds.coolant_storage,
			value: eva.coolant_storage
		}
	];
};

const suitAtmosphere = (eva: Astronaut): Metric[] => [
	{
		key: 'Heart Rate',
		shortKey: 'Heart Rate',
		units: AtmosphereBounds.heart_rate.units,
		formatter: twoDecimals,
		range: AtmosphereBounds.heart_rate,
		value: eva.heart_rate
	},
	{
		key: 'Oxygen Consumption',
		shortKey: 'O2 Use',
		units: AtmosphereBounds.oxy_consumption.units,
		formatter: twoDecimals,
		range: AtmosphereBounds.oxy_consumption,
		value: eva.oxy_consumption
	},
	{
		key: 'CO2 Production',
		shortKey: 'CO2 Prod',
		units: AtmosphereBounds.co2_production.units,
		formatter: twoDecimals,
		range: AtmosphereBounds.co2_production,
		value: eva.co2_production
	},
	{
		key: 'Suit Pressure Oxygen',
		shortKey: 'Suit O2',
		units: AtmosphereBounds.suit_pressure_oxy.units,
		formatter: twoDecimals,
		range: AtmosphereBounds.suit_pressure_oxy,
		value: eva.suit_pressure_oxy
	},
	{
		key: 'Suit Pressure CO2',
		shortKey: 'Suit CO2',
		units: AtmosphereBounds.suit_pressure_co2.units,
		formatter: twoDecimals,
		range: AtmosphereBounds.suit_pressure_co2,
		value: eva.suit_pressure_co2
	},
	{
		key: 'Suit Pressure Other',
		shortKey: 'Suit Other',
		units: AtmosphereBounds.suit_pressure_other.units,
		formatter: twoDecimals,
		range: AtmosphereBounds.suit_pressure_other,
		value: eva.suit_pressure_other
	},
	{
		key: 'Suit Pressure Total',
		shortKey: 'Suit Total',
		units: AtmosphereBounds.suit_pressure_total.units,
		formatter: twoDecimals,
		range: AtmosphereBounds.suit_pressure_total,
		value: eva.suit_pressure_total
	},
	{
		key: 'Helmet Pressure CO2',
		shortKey: 'Helmet CO2',
		units: AtmosphereBounds.helmet_pressure_co2.units,
		formatter: twoDecimals,
		range: AtmosphereBounds.helmet_pressure_co2,
		value: eva.helmet_pressure_co2
	}
];

const suitHelmet = (eva: Astronaut): Metric[] => [
	{
		key: 'Primary Fan Speed',
		shortKey: 'Pri Fan',
		units: HelmetBounds.fan_pri_rpm.units,
		formatter: twoDecimals,
		range: HelmetBounds.fan_pri_rpm,
		value: eva.fan_pri_rpm
	},
	{
		key: 'Secondary Fan Speed',
		shortKey: 'Sec Fan',
		units: HelmetBounds.fan_sec_rpm.units,
		formatter: twoDecimals,
		range: HelmetBounds.fan_sec_rpm,
		value: eva.fan_sec_rpm
	}
];

const suitScrubber = (eva: Astronaut): Metric[] => [
	{
		key: 'Scrubber A CO2 Storage',
		shortKey: 'Scrub A',
		units: ScrubberBounds.scrubber_a_co2_storage.units,
		formatter: twoDecimals,
		range: ScrubberBounds.scrubber_a_co2_storage,
		value: eva.scrubber_a_co2_storage
	},
	{
		key: 'Scrubber B CO2 Storage',
		shortKey: 'Scrub B',
		units: ScrubberBounds.scrubber_b_co2_storage.units,
		formatter: twoDecimals,
		range: ScrubberBounds.scrubber_b_co2_storage,
		value: eva.scrubber_b_co2_storage
	}
];

const suitTemperature = (eva: Astronaut): Metric[] => [
	{
		key: 'Temperature',
		shortKey: 'Temp',
		units: TemperatureBounds.temperature.units,
		formatter: twoDecimals,
		range: TemperatureBounds.temperature,
		value: eva.temperature
	},
	{
		key: 'Coolant Gas Pressure',
		shortKey: 'Cool Gas',
		units: TemperatureBounds.coolant_gas_pressure.units,
		formatter: twoDecimals,
		range: TemperatureBounds.coolant_gas_pressure,
		value: eva.coolant_gas_pressure
	},
	{
		key: 'Coolant Liquid Pressure',
		shortKey: 'Cool Liquid',
		units: TemperatureBounds.coolant_liquid_pressure.units,
		formatter: twoDecimals,
		range: TemperatureBounds.coolant_liquid_pressure,
		value: eva.coolant_liquid_pressure
	}
];

export const metricCategories = (eva: Astronaut | undefined): MetricCategoryMap => {
	if (!eva) return {};

	return {
		'Suit Resources': suitResources(eva),
		'Suit Atmosphere': suitAtmosphere(eva),
		'Suit Helmet Fan': suitHelmet(eva),
		'Suit CO2 Scrubbers': suitScrubber(eva),
		'Suit Temperature': suitTemperature(eva)
	};
};

export const flattenMetricCategories = (categories: MetricCategoryMap): FlatMetric[] =>
	Object.entries(categories).flatMap(([category, metrics]) =>
		metrics.map((metric) => ({
			...metric,
			category,
			categoryCode: categoryCodes[category] ?? 'SYS'
		}))
	);

export const metricStatus = (metric: Metric): MetricStatus => {
	if (metric.value < metric.range.min) return 'low';
	if (metric.value > metric.range.max) return 'high';
	return 'nominal';
};

export const issueCount = (metrics: Metric[]) =>
	metrics.filter((metric) => metricStatus(metric) !== 'nominal').length;

const clampPercent = (input: number) => Math.max(0, Math.min(input, 100));

export const metricPercent = (metric: Metric, value = metric.value) => {
	const [low, high] = metric.range.limit;
	const span = high - low;
	if (!Number.isFinite(span) || span === 0) return 0;
	return clampPercent(((value - low) / span) * 100);
};

export const metricStyle = (metric: Metric) => {
	return `--value:${metricPercent(metric)}%; --min:${metricPercent(metric, metric.range.min)}%; --max:${metricPercent(metric, metric.range.max)}%;`;
};
