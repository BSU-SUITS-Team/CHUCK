using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class EvaTelemetryView : MonoBehaviour
{
    [Header("Telemetry - EVA1")]
    [SerializeField] private TMP_Text eva1PrimaryBatteryText;
    [SerializeField] private TMP_Text eva1SecondaryBatteryText;
    [SerializeField] private TMP_Text eva1OxyPriStorageText;
    [SerializeField] private TMP_Text eva1OxySecStorageText;
    [SerializeField] private TMP_Text eva1OxyPriPressureText;
    [SerializeField] private TMP_Text eva1OxySecPressureText;
    [SerializeField] private TMP_Text eva1SuitPressureOxyText;
    [SerializeField] private TMP_Text eva1SuitPressureCo2Text;
    [SerializeField] private TMP_Text eva1SuitPressureOtherText;
    [SerializeField] private TMP_Text eva1SuitPressureTotalText;
    [SerializeField] private TMP_Text eva1HelmetPressureCo2Text;
    [SerializeField] private TMP_Text eva1FanPriRpmText;
    [SerializeField] private TMP_Text eva1FanSecRpmText;
    [SerializeField] private TMP_Text eva1ScrubberAText;
    [SerializeField] private TMP_Text eva1ScrubberBText;
    [SerializeField] private TMP_Text eva1TemperatureText;
    [SerializeField] private TMP_Text eva1CoolantStorageText;
    [SerializeField] private TMP_Text eva1CoolantGasPressureText;
    [SerializeField] private TMP_Text eva1CoolantLiquidPressureText;
    [SerializeField] private TMP_Text eva1HeartRateText;
    [SerializeField] private TMP_Text eva1OxyConsumptionText;
    [SerializeField] private TMP_Text eva1Co2ProductionText;
    [SerializeField] private TMP_Text eva1ElapsedTimeText;

    [Header("Telemetry - EVA2")]
    [SerializeField] private TMP_Text eva2BatteryText;
    [SerializeField] private TMP_Text eva2OxyPriStorageText;
    [SerializeField] private TMP_Text eva2OxySecStorageText;
    [SerializeField] private TMP_Text eva2OxyPriPressureText;
    [SerializeField] private TMP_Text eva2OxySecPressureText;
    [SerializeField] private TMP_Text eva2SuitPressureOxyText;
    [SerializeField] private TMP_Text eva2SuitPressureCo2Text;
    [SerializeField] private TMP_Text eva2SuitPressureOtherText;
    [SerializeField] private TMP_Text eva2SuitPressureTotalText;
    [SerializeField] private TMP_Text eva2HelmetPressureCo2Text;
    [SerializeField] private TMP_Text eva2FanPriRpmText;
    [SerializeField] private TMP_Text eva2FanSecRpmText;
    [SerializeField] private TMP_Text eva2ScrubberAText;
    [SerializeField] private TMP_Text eva2ScrubberBText;
    [SerializeField] private TMP_Text eva2TemperatureText;
    [SerializeField] private TMP_Text eva2CoolantStorageText;
    [SerializeField] private TMP_Text eva2CoolantGasPressureText;
    [SerializeField] private TMP_Text eva2CoolantLiquidPressureText;
    [SerializeField] private TMP_Text eva2HeartRateText;
    [SerializeField] private TMP_Text eva2OxyConsumptionText;
    [SerializeField] private TMP_Text eva2Co2ProductionText;
    [SerializeField] private TMP_Text eva2ElapsedTimeText;

    [Header("Status")]
    [SerializeField] private TMP_Text startedText;

    [Header("DCU - EVA1")]
    [SerializeField] private TMP_Text dcuEva1OxyText;
    [SerializeField] private TMP_Text dcuEva1FanText;
    [SerializeField] private TMP_Text dcuEva1PumpText;
    [SerializeField] private TMP_Text dcuEva1Co2Text;
    [SerializeField] private TMP_Text dcuEva1BattLuText;
    [SerializeField] private TMP_Text dcuEva1BattPsText;

    [Header("DCU - EVA2")]
    [SerializeField] private TMP_Text dcuEva2BattText;
    [SerializeField] private TMP_Text dcuEva2OxyText;
    [SerializeField] private TMP_Text dcuEva2CommText;
    [SerializeField] private TMP_Text dcuEva2FanText;
    [SerializeField] private TMP_Text dcuEva2PumpText;
    [SerializeField] private TMP_Text dcuEva2Co2Text;

    [Header("Error Flags")]
    [SerializeField] private TMP_Text fanErrorText;
    [SerializeField] private TMP_Text oxyErrorText;
    [SerializeField] private TMP_Text powerErrorText;
    [SerializeField] private TMP_Text scrubberErrorText;

    [Header("IMU")]
    [SerializeField] private TMP_Text imuEva1PosXText;
    [SerializeField] private TMP_Text imuEva1PosYText;
    [SerializeField] private TMP_Text imuEva1HeadingText;
    [SerializeField] private TMP_Text imuEva2PosXText;
    [SerializeField] private TMP_Text imuEva2PosYText;
    [SerializeField] private TMP_Text imuEva2HeadingText;

    [Header("UIA")]
    [SerializeField] private TMP_Text uiaEva1PowerText;
    [SerializeField] private TMP_Text uiaEva1OxyText;
    [SerializeField] private TMP_Text uiaEva1WaterSupplyText;
    [SerializeField] private TMP_Text uiaEva1WaterWasteText;
    [SerializeField] private TMP_Text uiaEva2PowerText;
    [SerializeField] private TMP_Text uiaEva2OxyText;
    [SerializeField] private TMP_Text uiaEva2WaterSupplyText;
    [SerializeField] private TMP_Text uiaEva2WaterWasteText;
    [SerializeField] private TMP_Text uiaOxyVentText;
    [SerializeField] private TMP_Text uiaDepressText;

    private TSSConnectionManager Manager => TSSConnectionManager.Instance;

    private void OnEnable()
    {
        if (Manager == null)
        {
            Debug.LogError("EvaTelemetryView: no TSSConnectionManager found.");
            return;
        }

        Manager.EvaUpdated += HandleEvaUpdated;

        if (Manager.EvaData != null)
            HandleEvaUpdated(Manager.EvaData);
    }

    private void OnDisable()
    {
        if (Manager != null)
            Manager.EvaUpdated -= HandleEvaUpdated;
    }

    private void HandleEvaUpdated(EvaRoot data)
    {
        if (data == null) return;

        EvaSuit eva1 = data.telemetry?.eva1;
        EvaSuit eva2 = data.telemetry?.eva2;

        SetFloat(eva1PrimaryBatteryText, eva1?.primary_battery_level);
        SetFloat(eva1SecondaryBatteryText, eva1?.secondary_battery_level);
        SetFloat(eva1OxyPriStorageText, eva1?.oxy_pri_storage);
        SetFloat(eva1OxySecStorageText, eva1?.oxy_sec_storage);
        SetFloat(eva1OxyPriPressureText, eva1?.oxy_pri_pressure);
        SetFloat(eva1OxySecPressureText, eva1?.oxy_sec_pressure);
        SetFloat(eva1SuitPressureOxyText, eva1?.suit_pressure_oxy);
        SetFloat(eva1SuitPressureCo2Text, eva1?.suit_pressure_co2);
        SetFloat(eva1SuitPressureOtherText, eva1?.suit_pressure_other);
        SetFloat(eva1SuitPressureTotalText, eva1?.suit_pressure_total);
        SetFloat(eva1HelmetPressureCo2Text, eva1?.helmet_pressure_co2);
        SetFloat(eva1FanPriRpmText, eva1?.fan_pri_rpm);
        SetFloat(eva1FanSecRpmText, eva1?.fan_sec_rpm);
        SetFloat(eva1ScrubberAText, eva1?.scrubber_a_co2_storage);
        SetFloat(eva1ScrubberBText, eva1?.scrubber_b_co2_storage);
        SetFloat(eva1TemperatureText, eva1?.temperature, "F1");
        SetFloat(eva1CoolantStorageText, eva1?.coolant_storage);
        SetFloat(eva1CoolantGasPressureText, eva1?.coolant_gas_pressure);
        SetFloat(eva1CoolantLiquidPressureText, eva1?.coolant_liquid_pressure);
        SetFloat(eva1HeartRateText, eva1?.heart_rate);
        SetFloat(eva1OxyConsumptionText, eva1?.oxy_consumption);
        SetFloat(eva1Co2ProductionText, eva1?.co2_production);
        SetFloat(eva1ElapsedTimeText, eva1?.eva_elapsed_time);

        float eva2Battery = 0f;
        if (eva2 != null)
            eva2Battery = eva2.battery_level != 0f ? eva2.battery_level : eva2.primary_battery_level;

        SetFloat(eva2BatteryText, eva2Battery);
        SetFloat(eva2OxyPriStorageText, eva2?.oxy_pri_storage);
        SetFloat(eva2OxySecStorageText, eva2?.oxy_sec_storage);
        SetFloat(eva2OxyPriPressureText, eva2?.oxy_pri_pressure);
        SetFloat(eva2OxySecPressureText, eva2?.oxy_sec_pressure);
        SetFloat(eva2SuitPressureOxyText, eva2?.suit_pressure_oxy);
        SetFloat(eva2SuitPressureCo2Text, eva2?.suit_pressure_co2);
        SetFloat(eva2SuitPressureOtherText, eva2?.suit_pressure_other);
        SetFloat(eva2SuitPressureTotalText, eva2?.suit_pressure_total);
        SetFloat(eva2HelmetPressureCo2Text, eva2?.helmet_pressure_co2);
        SetFloat(eva2FanPriRpmText, eva2?.fan_pri_rpm);
        SetFloat(eva2FanSecRpmText, eva2?.fan_sec_rpm);
        SetFloat(eva2ScrubberAText, eva2?.scrubber_a_co2_storage);
        SetFloat(eva2ScrubberBText, eva2?.scrubber_b_co2_storage);
        SetFloat(eva2TemperatureText, eva2?.temperature, "F1");
        SetFloat(eva2CoolantStorageText, eva2?.coolant_storage);
        SetFloat(eva2CoolantGasPressureText, eva2?.coolant_gas_pressure);
        SetFloat(eva2CoolantLiquidPressureText, eva2?.coolant_liquid_pressure);
        SetFloat(eva2HeartRateText, eva2?.heart_rate);
        SetFloat(eva2OxyConsumptionText, eva2?.oxy_consumption);
        SetFloat(eva2Co2ProductionText, eva2?.co2_production);
        SetFloat(eva2ElapsedTimeText, eva2?.eva_elapsed_time);

        SetBool(startedText, data.status?.started);

        SetBool(dcuEva1OxyText, data.dcu?.eva1?.oxy);
        SetBool(dcuEva1FanText, data.dcu?.eva1?.fan);
        SetBool(dcuEva1PumpText, data.dcu?.eva1?.pump);
        SetBool(dcuEva1Co2Text, data.dcu?.eva1?.co2);
        SetBool(dcuEva1BattLuText, data.dcu?.eva1?.batt?.lu);
        SetBool(dcuEva1BattPsText, data.dcu?.eva1?.batt?.ps);

        SetBool(dcuEva2BattText, data.dcu?.eva2?.batt);
        SetBool(dcuEva2OxyText, data.dcu?.eva2?.oxy);
        SetBool(dcuEva2CommText, data.dcu?.eva2?.comm);
        SetBool(dcuEva2FanText, data.dcu?.eva2?.fan);
        SetBool(dcuEva2PumpText, data.dcu?.eva2?.pump);
        SetBool(dcuEva2Co2Text, data.dcu?.eva2?.co2);

        SetBool(fanErrorText, data.error?.fan_error);
        SetBool(oxyErrorText, data.error?.oxy_error);
        SetBool(powerErrorText, data.error?.power_error);
        SetBool(scrubberErrorText, data.error?.scrubber_error);

        SetFloat(imuEva1PosXText, data.imu?.eva1?.posx, "F2");
        SetFloat(imuEva1PosYText, data.imu?.eva1?.posy, "F2");
        SetFloat(imuEva1HeadingText, data.imu?.eva1?.heading, "F1");
        SetFloat(imuEva2PosXText, data.imu?.eva2?.posx, "F2");
        SetFloat(imuEva2PosYText, data.imu?.eva2?.posy, "F2");
        SetFloat(imuEva2HeadingText, data.imu?.eva2?.heading, "F1");

        SetBool(uiaEva1PowerText, data.uia?.eva1_power);
        SetBool(uiaEva1OxyText, data.uia?.eva1_oxy);
        SetBool(uiaEva1WaterSupplyText, data.uia?.eva1_water_supply);
        SetBool(uiaEva1WaterWasteText, data.uia?.eva1_water_waste);
        SetBool(uiaEva2PowerText, data.uia?.eva2_power);
        SetBool(uiaEva2OxyText, data.uia?.eva2_oxy);
        SetBool(uiaEva2WaterSupplyText, data.uia?.eva2_water_supply);
        SetBool(uiaEva2WaterWasteText, data.uia?.eva2_water_waste);
        SetBool(uiaOxyVentText, data.uia?.oxy_vent);
        SetBool(uiaDepressText, data.uia?.depress);
    }

    private void SetFloat(TMP_Text target, float? value, string format = "F0")
    {
        if (target == null || value == null) return;
        target.text = value.Value.ToString(format);
    }

    private void SetBool(TMP_Text target, bool? value)
    {
        if (target == null || value == null) return;
        target.text = value.Value ? "ON" : "OFF";
    }
}