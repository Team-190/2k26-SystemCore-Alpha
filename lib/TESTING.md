# GompeiLib Testing Standards & Contract

This document defines the testing contract and standards for GompeiLib. All new features, refactors, and test suites must adhere to these standards.

## 1. Package Mirroring

Every class in `lib/src/main/java/.../Foo.java` must have its unit test located at the mirrored path in `lib/src/test/java/.../FooTest.java`.
- Example: `edu.wpi.team190.gompeilib.io.gyro.interfaces.GyroIO` -> `edu.wpi.team190.gompeilib.io.gyro.interfaces.GyroIOTest`
- Test classes must end in `Test` and match the class name under test exactly (including casing).
- This is automatically verified and enforced in CI by `TestContractEnforcerTest`.

## 2. Per-Category Test Contract

| Category | Example | Required coverage |
| --- | --- | --- |
| **IO interface** | `ExtensionIO`, `GyroIO` | Instantiate via anonymous class (`new FooIO() {}`), call every default method, assert documented defaults, including every `*Inputs` field's zero-value. |
| **Real hardware IO impl** | `ExtensionIOTalonFX`, `GyroIOPigeon2` | Never touch real CAN/hardware. Mock the vendor device via `Mockito.mockConstruction(...)`, mock batched signal refresh via `mockStatic(BaseStatusSignal.class)`. Assert config is applied, every accessor/input field maps correctly, and cover both the success path and at least one `StatusCode` failure/retry path. |
| **Sim IO impl** | `ExtensionIOSim`, `GenericRollerIOSim` | Drive the WPILib physics sim forward by known inputs/time steps; assert `updateInputs` reflects the expected physical state, no mocking of the sim math. |
| **Subsystem** | `Extension`, `SwerveDrive`, `GenericRoller` | Mock the IO layer completely. Assert `periodic()` behavior, command factories, state transitions, and logging calls. |
| **Constants / builder record** | `ExtensionConstants`, `VisionConstants` | Every field on the record/builder gets a distinct, non-default value that is asserted. Partial builder coverage is not acceptable. |
| **State / mode enum** | `ExtensionState`, `RobotMode` | Every enum constant and any attached lookup/transition logic. |
| **Pure utility / math** | `GeometryUtil`, `LinearProfile`, `Gains`, `MeasureUtil` | No mocking. Cover nominal, zero, negative, and boundary inputs explicitly. |
| **Data record / observation** | `VisionPoseObservation`, `VisionMultiTxTyObservation` | Constructor/accessor values and `equals`/`hashCode` if overridden. |
| **Robot harness scenario** | Full-match, binding, and auto scenarios in the robot project | Drives the real `Robot` through the JUnit harness (`DriverStationSim`, joystick sims, stepped timing); asserts on device sim state and the ground-truth pose with named tolerances; checks for exceptions, command conflicts, and alerts (#47). |

## 3. Shared Test Fixtures & Lifecycle

Tests requiring WPILib HAL or GompeiLib state initialization must extend `GompeiLibTestBase`:
- Starts HAL using `assert HAL.initialize()` per WPILib unit testing documentation.
- Handles `GompeiLib.init(RobotMode.SIM, isTuningMode(), loopPeriodSecs())` and cleans up in `@AfterEach`.
- Automatically closes any `AutoCloseable` resources registered via `registerCloseable(resource)`.
- Never call raw `HAL.initialize()`, `GompeiLib.init()`, or `GompeiLib.deinit()` directly in test classes; this is enforced by `TestContractEnforcerTest`.

## 4. Standardized Mocking

- Use Mockito only.
- Hardware device construction is mocked via `mockConstruction(...)`.
- Static/batched calls (e.g. `BaseStatusSignal.setUpdateFrequencyForAll`, `Logger.recordOutput`) must be mocked via `MockedStatic` scoped within try-with-resources blocks.
- Hardware mocks must never attempt to communicate with real CAN buses or physical hardware in test runs.

## 5. Naming Convention

- Test method names must start with `test` or `should` (e.g., `testVelocityControl()`, `shouldRejectInvalidAngle()`). Bare method names (e.g., `defaultValueTuningMode()`) are not allowed and are rejected by CI.
- Test class names must be `<ClassUnderTest>Test`.

## 6. Coverage Enforcement

- 100% line/branch coverage is tracked and enforced.
- Additions to `jacocoTestCoverageVerification` exclusions (`**/*TalonFX.class`, `**/*Pigeon2.class`, etc.) are reserved strictly for hardware-dependent logic untestable via mocks/sims and must be documented and justified.
