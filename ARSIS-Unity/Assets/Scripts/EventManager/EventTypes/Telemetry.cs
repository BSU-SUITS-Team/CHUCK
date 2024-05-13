using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ARSIS.EventManager
{
	[System.Serializable]
	public class TelemetryEva
	{
		public float batt_time_left { get; set; }
		public float oxy_pri_storage { get; set; }
		public float oxy_sec_storage { get; set; }
		public float oxy_pri_pressure { get; set; }
		public float oxy_sec_pressure { get; set; }
		public float oxy_time_left { get; set; }
		public float heart_rate { get; set; }
		public float oxy_consumption { get; set; }
		public float co2_production { get; set; }
		public float suit_pressure_oxy { get; set; }
		public float suit_pressure_co2 { get; set; }
		public float suit_pressure_other { get; set; }
		public float suit_pressure_total { get; set; }
		public float fan_pri_rpm { get; set; }
		public float fan_sec_rpm { get; set; }
		public float helmet_pressure_co2 { get; set; }
		public float scrubber_a_co2_storage { get; set; }
		public float scrubber_b_co2_storage { get; set; }
		public float temperature { get; set; }
		public float coolant_ml { get; set; }
		public float coolant_gas_pressure { get; set; }
		public float coolant_liquid_pressure { get; set; }
	}

	[System.Serializable]
	public class TelemetryData
	{
		public int eva_time { get; set; }
		public TelemetryEva eva1 { get; set; }
		public TelemetryEva eva2 { get; set; }
	}

	[System.Serializable]
    public class Telemetry : BaseArsisEvent
    {
		public TelemetryData data { get; set; }

        public override string ToString()
        {
			return "Telemetry event";
        }
    }
}
