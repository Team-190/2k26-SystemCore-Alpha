package edu.wpi.team190.gompeilib.contract;

import static org.junit.jupiter.api.Assertions.*;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Method;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

public class TestContractEnforcerTest {

  private static final Path MAIN_DIR = Path.of("src/main/java");
  private static final Path TEST_DIR = Path.of("src/test/java");

  // Main files that are explicitly excluded from test mirroring (e.g., AspectJ aspects, unported
  // legacy tracked in follow-up issues)
  private static final Set<String> MIRROR_EXCLUDED_MAIN =
      Set.of(
          "edu/wpi/team190/gompeilib/logging/TracerAspect.java",
          // CameraStaticLimelight and CameraMovingLimelight are tested in CameraLimelightTest until
          // refactored in #39
          "edu/wpi/team190/gompeilib/robot/subsystems/vision/camera/CameraStaticLimelight.java",
          "edu/wpi/team190/gompeilib/robot/subsystems/vision/camera/CameraMovingLimelight.java");

  // Shared test fixtures or meta-tests that don't correspond to a single main class
  private static final Set<String> MIRROR_EXCLUDED_TEST =
      Set.of(
          "edu/wpi/team190/gompeilib/GompeiLibTestBase.java",
          "edu/wpi/team190/gompeilib/contract/TestContractEnforcerTest.java",
          // Shared reflection helpers for the core/io contract and Phoenix6 coverage tests
          "edu/wpi/team190/gompeilib/io/IOTestSupport.java",
          // Shared motor fixture and all-capability test mechanism for the mechanism-layer tests
          "edu/wpi/team190/gompeilib/robot/mechanisms/MechanismTestSupport.java",
          // Joint test for StaticLimelight and MovingLimelight
          "edu/wpi/team190/gompeilib/robot/subsystems/vision/camera/CameraLimelightTest.java");

  private Path resolveDir(Path rel) {
    if (Files.exists(rel)) {
      return rel;
    }
    Path sub = Path.of("lib").resolve(rel);
    if (Files.exists(sub)) {
      return sub;
    }
    throw new IllegalStateException("Cannot find directory: " + rel);
  }

  @Test
  public void testPackageMirroring() throws IOException {
    Path mainBase = resolveDir(MAIN_DIR);
    Path testBase = resolveDir(TEST_DIR);

    List<String> mainFiles;
    try (Stream<Path> stream = Files.walk(mainBase)) {
      mainFiles =
          stream
              .filter(p -> p.toString().endsWith(".java"))
              .map(p -> mainBase.relativize(p).toString().replace(File.separatorChar, '/'))
              .filter(f -> !MIRROR_EXCLUDED_MAIN.contains(f))
              .collect(Collectors.toList());
    }

    List<String> testFiles;
    try (Stream<Path> stream = Files.walk(testBase)) {
      testFiles =
          stream
              .filter(p -> p.toString().endsWith(".java"))
              .map(p -> testBase.relativize(p).toString().replace(File.separatorChar, '/'))
              .filter(f -> !MIRROR_EXCLUDED_TEST.contains(f))
              .collect(Collectors.toList());
    }

    Set<String> testFileSet = new HashSet<>(testFiles);
    List<String> missingTests = new ArrayList<>();
    for (String mainFile : mainFiles) {
      String expectedTest = mainFile.replace(".java", "Test.java");
      if (!testFileSet.contains(expectedTest)) {
        missingTests.add("Main class " + mainFile + " has no mirrored test at " + expectedTest);
      }
    }

    Set<String> mainFileSet = new HashSet<>(mainFiles);
    List<String> orphanTests = new ArrayList<>();
    for (String testFile : testFiles) {
      if (!testFile.endsWith("Test.java")) {
        orphanTests.add("Test file does not end in Test.java: " + testFile);
        continue;
      }
      String expectedMain =
          testFile.substring(0, testFile.length() - "Test.java".length()) + ".java";
      if (!mainFileSet.contains(expectedMain)) {
        orphanTests.add("Test file " + testFile + " has no mirrored main class at " + expectedMain);
      }
    }

    assertTrue(
        missingTests.isEmpty() && orphanTests.isEmpty(),
        "Package mirroring violations:\nMissing tests:\n"
            + String.join("\n", missingTests)
            + "\nOrphan tests:\n"
            + String.join("\n", orphanTests));
  }

  @Test
  public void testTestMethodNaming() throws IOException, ClassNotFoundException {
    Path testBase = resolveDir(TEST_DIR);
    List<String> testClassNames;
    try (Stream<Path> stream = Files.walk(testBase)) {
      testClassNames =
          stream
              .filter(p -> p.toString().endsWith(".java"))
              .map(p -> testBase.relativize(p).toString().replace(File.separatorChar, '/'))
              .filter(f -> f.endsWith("Test.java"))
              .map(f -> f.replace(".java", "").replace('/', '.'))
              .collect(Collectors.toList());
    }

    List<String> violations = new ArrayList<>();
    for (String className : testClassNames) {
      Class<?> clazz = Class.forName(className);
      for (Method method : clazz.getDeclaredMethods()) {
        if (method.isAnnotationPresent(Test.class)) {
          String name = method.getName();
          if (!name.startsWith("test") && !name.startsWith("should")) {
            violations.add(className + "#" + name + " does not start with 'test' or 'should'");
          }
        }
      }
    }

    assertTrue(
        violations.isEmpty(),
        "Test method naming violations (must start with 'test' or 'should'):\n"
            + String.join("\n", violations));
  }

  @Test
  public void testNoRawLifecycleCallsOutsideFixture() throws IOException {
    Path testBase = resolveDir(TEST_DIR);

    // Files allowed to manage lifecycle directly
    Set<String> allowedFiles =
        Set.of(
            "edu/wpi/team190/gompeilib/GompeiLibTestBase.java",
            "edu/wpi/team190/gompeilib/GompeiLibTest.java",
            "edu/wpi/team190/gompeilib/contract/TestContractEnforcerTest.java");

    List<String> violations = new ArrayList<>();
    try (Stream<Path> stream = Files.walk(testBase)) {
      List<Path> files =
          stream.filter(p -> p.toString().endsWith(".java")).collect(Collectors.toList());

      for (Path file : files) {
        String relPath = testBase.relativize(file).toString().replace(File.separatorChar, '/');
        if (allowedFiles.contains(relPath)) {
          continue;
        }

        List<String> lines = Files.readAllLines(file);
        for (int i = 0; i < lines.size(); i++) {
          String line = lines.get(i);
          // Check for HAL.initialize
          if (line.contains("HAL.initialize")) {
            violations.add(relPath + ":" + (i + 1) + " calls HAL.initialize directly");
          }
          // Check for GompeiLib.deinit()
          if (line.contains("GompeiLib.deinit(")) {
            violations.add(relPath + ":" + (i + 1) + " calls GompeiLib.deinit() directly");
          }
          // Check for GompeiLib.init(
          if (line.contains("GompeiLib.init(")) {
            violations.add(relPath + ":" + (i + 1) + " calls GompeiLib.init() directly");
          }
        }
      }
    }

    assertTrue(
        violations.isEmpty(),
        "Raw HAL/GompeiLib lifecycle calls found outside shared fixture:\n"
            + String.join("\n", violations));
  }
}
