package frc.robot.shuffleboard;

import java.util.ArrayList;

import edu.wpi.first.networktables.GenericEntry;

import frc.robot.autos.AutoSelector;

/** Controls and initializes Shuffleboard tabs. */
public class ShuffleboardManager {
  /** If used, initializes test tabs. */
  public static final boolean m_testing = true;

  private static ShuffleboardManager m_instance;

  /** @return the singleton instance */
  public static synchronized ShuffleboardManager getInstance() {
    if (m_instance == null) {
      m_instance = new ShuffleboardManager();
    }
    return m_instance;
  }

  public interface ShuffleboardTabBase {
    /** Creates all widgets. */
    public void initialize();
  }

  private static final ArrayList<ShuffleboardTabBase> m_tabs = new ArrayList<ShuffleboardTabBase>();

  private ShuffleboardManager() {
    m_tabs.add(new DriveTab());
    if (m_testing) {
      m_tabs.add(new TestTab());
      m_tabs.add(new SwerveTab());
    }

    m_tabs.forEach(e -> e.initialize());
  }

  /** Shuffleboard entry. Should be true if a cube is preloaded. */
  protected static GenericEntry cubeLoaded;

  /** Shuffleboard entry. Should be false if any chooser needs to be updated. */
  protected static GenericEntry choosersSynced;

  public boolean getCubePreloaded() {
    if (cubeLoaded != null) {
      return cubeLoaded.getBoolean(true);
    }
    return true;
  }

  public void updateChoosersSynced() {
    if (choosersSynced != null) {
      choosersSynced.setBoolean(
        !(AutoSelector.m_bodyChooser.isUpdateReq() ||
        AutoSelector.m_secondaryChooser.isUpdateReq() ||
        AutoSelector.m_endingChooser.isUpdateReq())
      );
    }
  }
}