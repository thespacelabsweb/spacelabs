# Guidance System Function Flow Chart

## System Overview

This flowchart represents the function call hierarchy and data flow for the onboard guidance algorithm system.

## Main System Flow

```mermaid
flowchart TD
    A[Start] --> B[Initialize System Parameters]
    B --> C[Set Launch Point Coordinates]
    C --> D[Set Target Coordinates]
    D --> E[Set Initial Projectile States]
    E --> F[Call Main Function]

    F --> G[Initialize Guidance Parameters]
    G --> H[Convert Target Coordinates to ECI Frame]
    H --> I[Main Guidance Loop Start]

    I --> J{Check Loop Conditions}
    J -->|Continue| K[Convert ECI Frame to Local Frame]
    J -->|Exit| Z[End Guidance]

    K --> L[Convert Velocity to Local Frame]
    L --> M[Calculate Current States]
    M --> N[Calculate Vector Coordinates to Target Cordinates]
    N --> O[Calculate Distance and Unit Vectors]

    O --> P{Check Terminal Phase}
    P -->|Terminal| Q[Set Zero Acceleration]
    P -->|Guided| R[Calculate Guidance Commands]

    R --> S[Calculate Angular Velocity]
    S --> T[Calculate Quaternion Parameters]
    T --> U[Apply Quaternion Transformations]
    U --> V[Calculate Impact Parameters]
    V --> W[Calculate Time-to-Go]
    W --> X[Calculate Final Acceleration Commands]

    Q --> Y[Convert to ECI Frame]
    X --> Y
    Y --> AA[Calculate Drag Forces]
    AA --> BB[Calculate Gravity Effects]
    BB --> CC[Integrate Acceleration to Velocity]
    CC --> DD[Integrate Velocity to Position]
    DD --> EE[Update Time and Distance]
    EE --> I
```

## Coordinate Transformation Subsystem

```mermaid
flowchart TD
    A1[Coordinate Input] --> B1{Transformation Type}

    B1 -->|Geodetic to ECI| C1[geodeticToECI]
    B1 -->|ECI to Geodetic| C2[eciToGeodetic]
    B1 -->|ECI to Local| C3[ECI_to_Local]
    B1 -->|Local to ECI| C4[Local_to_ECI]
    B1 -->|Body to Local| C5[b2l]
    B1 -->|Local to Body| C6[l2b]

    C1 --> D1[Apply WGS84 Parameters]
    C2 --> D2[Iterative Latitude Calculation]
    C3 --> D3[Compute Local Frame Basis]
    C4 --> D4[Apply Rotation Matrix]
    C5 --> D5[Apply Euler Rotation Matrix]
    C6 --> D6[Apply Inverse Euler Matrix]

    D1 --> E1[Output ECI Coordinates]
    D2 --> E2[Output Geodetic Coordinates]
    D3 --> E3[Output Local Coordinates]
    D4 --> E4[Output ECI Coordinates]
    D5 --> E5[Output Local Coordinates]
    D6 --> E6[Output Body Coordinates]
```

## Guidance Law Subsystem

```mermaid
flowchart TD
    A2[Guidance Input] --> B2[Calculate Current Angles]
    B2 --> C2[Calculate Look Angle]
    C2 --> D2[Calculate Angular Velocity]
    D2 --> E2[Calculate Quaternion Parameters]

    E2 --> F2[Apply Quaternion Transform]
    F2 --> G2[Calculate Impact Angle Error]
    G2 --> H2[Calculate Time-to-Go]
    H2 --> I2[Apply Sign Function]
    I2 --> J2[Calculate Final Commands]

    J2 --> K2[Output Acceleration Commands]
```

## Physics Calculation Subsystem

```mermaid
flowchart TD
    A3[Physics Input] --> B3[Calculate Dynamic Pressure]
    B3 --> C3[Calculate Drag Force]
    C3 --> D3[Calculate Drag Acceleration]
    D3 --> E3[Calculate Gravity Vector]
    E3 --> F3[Sum Total Forces]
    F3 --> G3[Integrate Motion]
    G3 --> H3[Update States]
```

## Function Dependencies

### Primary Functions

- **Test_OBG.m** - Main test driver
- **onboard_guidance_algorithm_3** - Core guidance algorithm

### Coordinate Transformation Functions

- **geodeticToECI** - Geodetic to ECI conversion
- **eciToGeodetic** - ECI to geodetic conversion
- **ECI_to_Local** - ECI to local frame conversion
- **ECI_to_Local_vel** - ECI velocity to local frame
- **Local_to_ECI** - Local to ECI frame conversion
- **Local_to_ECI_vel** - Local velocity to ECI frame
- **b2l** - Body to local frame conversion
- **l2b** - Local to body frame conversion
- **computeRotationMatrix** - Rotation matrix computation

### Utility Functions

- **sig** - Sign function with power operation

## Data Flow Summary

1. **Initialization Phase**

   - Set system parameters and constants
   - Initialize coordinate frames
   - Set initial conditions

2. **Main Guidance Loop**

   - Coordinate transformations
   - State calculations
   - Guidance law application
   - Physics integration

3. **Output Phase**
   - Final position and velocity
   - Performance metrics
   - System status

## System Interfaces

### Input Interfaces

- Launch point coordinates (lat, lon, alt)
- Target coordinates (lat, lon, alt)
- Initial projectile states (position, velocity)
- Guidance parameters

### Output Interfaces

- Final projectile states
- Guidance commands
- System performance data
- Status information

### Internal Interfaces

- Coordinate transformation functions
- Guidance law calculations
- Physics integration functions
- Utility functions
