import { expect, test } from "vitest";
import { type TelemetryEvent, getAstronauts, getEVA, compareValueToBounds, ResourceBounds, HelmetBounds, AtmosphereBounds, ScrubberBounds, TemperatureBounds, Bounds } from "../src/lib/biometrics";

const sampleTelemetry: TelemetryEvent = {
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

test("getAstronauts() returns [\"eva1\", \"eva2\"]", () => {
    expect(getAstronauts(sampleTelemetry)).toEqual(["eva1", "eva2"]);
});
test("getEVA(eva1) returns eva2", () => {
    const { telemetry } = sampleTelemetry;
    const { eva2 } = telemetry;
    expect(getEVA(sampleTelemetry, "eva2")).toEqual(eva2);
});
test("getEVA(eva1) returns eva1", () => {
    const { telemetry } = sampleTelemetry;
    const { eva1 } = telemetry;
    expect(getEVA(sampleTelemetry, "eva1")).toEqual(eva1);
});
test("getEVA(eva1) returns eva2", () => {
    const { telemetry } = sampleTelemetry;
    const { eva2 } = telemetry;
    expect(getEVA(sampleTelemetry, "eva2")).toEqual(eva2);
});
test("getEVA(nil) returns undefined", () => {
    expect(getEVA(sampleTelemetry, "nil")).toEqual(undefined);
});
test("compareValueToBounds(1000, ResourceBounds.oxy_pri_pressure) returns Nominal", () => {
    expect(compareValueToBounds(1000, ResourceBounds.oxy_pri_pressure)).toBe(Bounds.Nominal);
});
test("compareValueToBounds(140, AtmosphereBounds.heart_rate) returns Nominal", () => {
    expect(compareValueToBounds(140, AtmosphereBounds.heart_rate)).toBe(Bounds.Nominal);
});
test("compareValueToBounds(60_000, HelmetBounds.fan_pri_rpm) returns Max", () => {
    expect(compareValueToBounds(60_000, HelmetBounds.fan_pri_rpm)).toBe(Bounds.Max);
});
test("compareValueToBounds(80, ScrubberBounds.scrubber_b_co2_storage) returns Max", () => {
    expect(compareValueToBounds(80, ScrubberBounds.scrubber_b_co2_storage)).toBe(Bounds.Max);
});
test("compareValueToBounds(32, TemperatureBounds.temperature) returns Min", () => {
    expect(compareValueToBounds(32, TemperatureBounds.temperature)).toBe(Bounds.Min);
});
test("compareValueToBounds(79, ResourceBounds.coolant_ml) returns Min", () => {
    expect(compareValueToBounds(79, ResourceBounds.coolant_ml)).toBe(Bounds.Min);
});