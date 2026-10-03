package frc190.gradle;

import org.gradle.api.Plugin;
import org.gradle.api.Project;
import org.gradle.api.artifacts.type.ArtifactTypeDefinition;
import org.gradle.api.attributes.Attribute;

/**
 * Works around a PathPlannerLib bug on WPILib 2027 alpha-7.
 *
 * <p>In alpha-7, {@code Alert(String, String, Level)} changed from {@code (group, text, level)} to
 * {@code (id, text, level)}, and alert ids must be unique. PathPlannerLib 2027.0.0-alpha-4 still
 * creates all of its RobotConfig alerts as {@code new Alert("PathPlanner", text, level)}, so the
 * second one throws "Alert already allocated" and RobotConfig fails to load (on the robot, in sim,
 * and in tests).
 *
 * <p>This plugin registers an artifact transform that rewrites those calls in the PathPlannerLib
 * jar to {@code new Alert("PathPlanner", text, text, level)} (group, id, text, level) on every
 * runtime classpath. Once PathPlanner ships a fix the transform finds nothing to patch and does
 * nothing, so it is safe to leave in, but it should be removed then.
 */
public class PathPlannerAlertPatchPlugin implements Plugin<Project> {
  public static final Attribute<Boolean> PATCHED =
      Attribute.of("frc190.pathplannerAlertPatched", Boolean.class);

  @Override
  public void apply(Project project) {
    var dependencies = project.getDependencies();
    dependencies.getAttributesSchema().attribute(PATCHED);
    dependencies
        .getArtifactTypes()
        .maybeCreate(ArtifactTypeDefinition.JAR_TYPE)
        .getAttributes()
        .attribute(PATCHED, false);
    dependencies.registerTransform(
        PathPlannerAlertPatchTransform.class,
        spec -> {
          spec.getFrom()
              .attribute(
                  ArtifactTypeDefinition.ARTIFACT_TYPE_ATTRIBUTE, ArtifactTypeDefinition.JAR_TYPE)
              .attribute(PATCHED, false);
          spec.getTo()
              .attribute(
                  ArtifactTypeDefinition.ARTIFACT_TYPE_ATTRIBUTE, ArtifactTypeDefinition.JAR_TYPE)
              .attribute(PATCHED, true);
        });

    // Only runtime classpaths need the fix (deploy, simulation and tests all run from one).
    project
        .getConfigurations()
        .configureEach(
            configuration -> {
              if (configuration.isCanBeResolved()
                  && configuration.getName().toLowerCase().endsWith("runtimeclasspath")) {
                configuration.getAttributes().attribute(PATCHED, true);
              }
            });
  }
}
