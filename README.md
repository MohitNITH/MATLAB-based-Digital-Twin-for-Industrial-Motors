# Induction Motor Fault Diagnosis Simulation

![MATLAB](https://img.shields.io/badge/Made%20with-MATLAB-0076A8?style=for-the-badge&logo=mathworks)

A comprehensive MATLAB simulation designed to model the dynamic behavior of an industrial induction motor under various fault conditions. This project generates a high-fidelity, labeled dataset intended for developing and testing predictive maintenance and Fault Detection and Diagnosis (FDD) algorithms.

This simulation was developed as part of a summer internship project at the **Delhi Metro Rail Corporation (DMRC)** [cite: 1, 2, 7] [cite_start]and is based on the specifications of a 37kW tunnel booster fan motor.

## 📋 Key Features

* **Dynamic Motor Modeling**: Simulates a 37kW, 3-phase induction motor.
* **Multiple Fault Scenarios**: Generates data for normal operation and 11 distinct electrical and mechanical fault types.
* **Multi-Sensor Data Generation**: Creates time-series data for 3-phase currents and voltages, RPM, temperature, 3-axis vibration, and acoustic noise.
* **Automated Visualization**: Automatically produces comparison plots for each fault against the normal baseline.
* **Statistical Summary**: Outputs a CSV file with key statistical metrics (mean, std, min, max) for all simulated conditions.

## 🔩 Simulated Fault Conditions

The simulation generates data for the following 12 conditions (1 normal + 11 faults):

1.  Normal Operation
2.  Broken Rotor Bar
3.  Bearing Damage (Outer Race)
4.  Bearing Damage (Inner Race)
5.  Bearing Damage (Rolling Element)
6.  Stator Winding Short
7.  Stator Winding Open
8.  Voltage Unbalance
9.  Rotor Eccentricity
10. Phase Loss
11. Belt Misalignment
12. Mechanical Unbalance

## 📂 Project Structure

After running the simulation, the following structure will be created:

```
.
├── fault_comparisons/
│   ├── bearing_damage_inner_race_comparison.png
│   ├── broken_rotor_bar_comparison.png
│   └── ... (plots for all 11 faults)
│
├── motor_fault_data/
│   ├── normal_operation.csv
│   ├── bearing_damage_inner_race.csv
│   └── ... (data for all 12 conditions)
│
├── fault_statistics.csv
└── Sim7.m
```

The CSV files in `motor_fault_data/` contain the following columns:
`Time, RPM, Current_PhaseA, Current_PhaseB, Current_PhaseC, Voltage_PhaseA, Voltage_PhaseB, Voltage_PhaseC, Temperature, Vibration_X, Vibration_Y, Vibration_Z, Acoustic`

## 🚀 How to Run

1.  Clone this repository to your local machine.
2.  Open MATLAB.
3. Run the `Sim7.m` script.

The script will automatically simulate all conditions and generate the `motor_fault_data`, `fault_comparisons`, and `fault_statistics.csv` files in the project directory.

## 📊 Example Visualizations

The simulation provides clear visual comparisons between normal and faulty states.

### Phase Loss Fault
A phase loss fault shows a dramatic drop in motor RPM and a significant increase in temperature.

![Phase Loss Comparison](phase_loss_comparison.png)

### Bearing Damage (Inner Race)
Inner race bearing damage introduces periodic, high-frequency components into the vibration signal and increases operating temperature.

![Bearing Damage Inner Race Comparison](bearing_damage_inner_race_comparison.png)

### Mechanical Unbalance
Mechanical unbalance significantly increases the amplitude of vibrations at the motor's running speed (1x RPM).

![Mechanical Unbalance Comparison](mechanical_unbalance_comparison.png)

## Acknowledgements

This work was conducted as part of the Summer Internship Program (27th May '25 to 24th June '25) at the Delhi Metro Rail Corporation (DMRC), in partial fulfillment of the requirements for the Bachelor of Technology degree from the National Institute of Technology, Hamirpur.


