Import("env")
import os

# Workaround for micro-ROS with paths containing spaces
# Set proper quoting for git operations
env.Replace(
    GIT="git -c core.longpaths=true"
)

# Ensure library builds use short paths
lib_dir = os.path.join(env.subst("$PROJECT_DIR"), ".pio", "libdeps", env.subst("$PIOENV"))
if " " in lib_dir:
    print("Warning: Project path contains spaces. This may cause issues with micro-ROS.")
    print("Consider moving project to a path without spaces.")
