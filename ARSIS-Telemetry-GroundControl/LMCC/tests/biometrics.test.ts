import { expect, test } from "vitest";
import { type TelemetryEvent, getAstronauts, getEVA, compareValueToBounds, ResourceBounds, HelmetBounds, AtmosphereBounds, ScrubberBounds, TemperatureBounds, Bounds, sampleTelemetry } from "../src/lib/biometrics";

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