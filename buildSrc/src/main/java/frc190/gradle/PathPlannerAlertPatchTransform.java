package frc190.gradle;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;
import java.util.zip.ZipOutputStream;
import org.gradle.api.artifacts.transform.InputArtifact;
import org.gradle.api.artifacts.transform.TransformAction;
import org.gradle.api.artifacts.transform.TransformOutputs;
import org.gradle.api.artifacts.transform.TransformParameters;
import org.gradle.api.file.FileSystemLocation;
import org.gradle.api.logging.Logging;
import org.gradle.api.provider.Provider;
import org.gradle.api.tasks.PathSensitive;
import org.gradle.api.tasks.PathSensitivity;
import org.objectweb.asm.ClassReader;
import org.objectweb.asm.ClassWriter;
import org.objectweb.asm.Opcodes;
import org.objectweb.asm.tree.AbstractInsnNode;
import org.objectweb.asm.tree.ClassNode;
import org.objectweb.asm.tree.FieldInsnNode;
import org.objectweb.asm.tree.LdcInsnNode;
import org.objectweb.asm.tree.MethodInsnNode;
import org.objectweb.asm.tree.MethodNode;

/** See {@link PathPlannerAlertPatchPlugin}. Jars other than PathPlannerLib pass through as-is. */
public abstract class PathPlannerAlertPatchTransform
    implements TransformAction<TransformParameters.None> {
  private static final String ALERT = "org/wpilib/util/Alert";
  private static final String LEVEL = "org/wpilib/util/Alert$Level";
  private static final String ID_TEXT_LEVEL =
      "(Ljava/lang/String;Ljava/lang/String;L" + LEVEL + ";)V";
  private static final String GROUP_ID_TEXT_LEVEL =
      "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;L" + LEVEL + ";)V";

  @InputArtifact
  @PathSensitive(PathSensitivity.NAME_ONLY)
  public abstract Provider<FileSystemLocation> getInputArtifact();

  @Override
  public void transform(TransformOutputs outputs) {
    File input = getInputArtifact().get().getAsFile();
    if (!input.getName().startsWith("PathplannerLib-java-")) {
      outputs.file(input);
      return;
    }

    File output = outputs.file(input.getName().replace(".jar", "-alertpatched.jar"));
    int patched = 0;
    try (ZipFile in = new ZipFile(input);
        ZipOutputStream out = new ZipOutputStream(new FileOutputStream(output))) {
      var entries = in.entries();
      while (entries.hasMoreElements()) {
        ZipEntry entry = entries.nextElement();
        byte[] bytes;
        try (InputStream stream = in.getInputStream(entry)) {
          bytes = stream.readAllBytes();
        }
        if (entry.getName().endsWith(".class")) {
          ClassNode classNode = new ClassNode();
          new ClassReader(bytes).accept(classNode, 0);
          int count = patchAlerts(classNode);
          if (count > 0) {
            ClassWriter writer = new ClassWriter(ClassWriter.COMPUTE_MAXS);
            classNode.accept(writer);
            bytes = writer.toByteArray();
            patched += count;
          }
        }
        ZipEntry copy = new ZipEntry(entry.getName());
        copy.setTime(entry.getTime());
        out.putNextEntry(copy);
        out.write(bytes);
        out.closeEntry();
      }
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
    Logging.getLogger(PathPlannerAlertPatchTransform.class)
        .info("Patched {} PathPlanner Alert constructor call(s) in {}", patched, input.getName());
  }

  /**
   * Rewrites {@code new Alert(group, text, level)} (where both strings are constants) into {@code
   * new Alert(group, text, text, level)}, i.e. the (group, id, text, level) constructor.
   */
  private static int patchAlerts(ClassNode classNode) {
    int count = 0;
    for (MethodNode method : classNode.methods) {
      for (AbstractInsnNode insn : method.instructions.toArray()) {
        if (!(insn instanceof MethodInsnNode call)
            || call.getOpcode() != Opcodes.INVOKESPECIAL
            || !call.owner.equals(ALERT)
            || !call.name.equals("<init>")
            || !call.desc.equals(ID_TEXT_LEVEL)) {
          continue;
        }
        AbstractInsnNode level = call.getPrevious();
        AbstractInsnNode text = level == null ? null : level.getPrevious();
        AbstractInsnNode group = text == null ? null : text.getPrevious();
        if (level instanceof FieldInsnNode levelField
            && levelField.getOpcode() == Opcodes.GETSTATIC
            && levelField.owner.equals(LEVEL)
            && text instanceof LdcInsnNode textLdc
            && textLdc.cst instanceof String textValue
            && group instanceof LdcInsnNode groupLdc
            && groupLdc.cst instanceof String) {
          method.instructions.insert(text, new LdcInsnNode(textValue));
          call.desc = GROUP_ID_TEXT_LEVEL;
          count++;
        }
      }
    }
    return count;
  }
}
