#!/usr/bin/env bash
# Run in the Ubuntu VM. No root access and no robot motion commands.
set -euo pipefail
rviz_package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
rviz_data_dir="${XDG_DATA_HOME:-$HOME/.local/share}/robot_navigation"
rviz_config_dir="${XDG_CONFIG_HOME:-$HOME/.config}"
rviz_application_dir="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
test -f "$rviz_package_dir/config/navigation.rviz"
test -f /opt/ros/humble/setup.bash
backup_if_present() {
  if [[ -e "$1" ]]; then
    # Read before replacing and retain the user's exact previous file.
    head -n 4 -- "$1"
    cp -p -- "$1" "$1.backup-$(date +%Y%m%dT%H%M%S%N)"
  fi
}
mkdir -p "$rviz_data_dir" "$rviz_config_dir/autostart" "$rviz_application_dir" "$HOME/.rviz2"
for rviz_target in "$rviz_data_dir/navigation.rviz" "$rviz_data_dir/start_rviz.sh" \
    "$HOME/.rviz2/default.rviz" "$rviz_application_dir/robot-navigation-rviz.desktop" \
    "$rviz_config_dir/autostart/robot-navigation-rviz.desktop"; do
  backup_if_present "$rviz_target"
done
install -m 0644 "$rviz_package_dir/config/navigation.rviz" "$rviz_data_dir/navigation.rviz"
install -m 0755 "$rviz_package_dir/scripts/start_rviz.sh" "$rviz_data_dir/start_rviz.sh"
install -m 0644 "$rviz_package_dir/config/navigation.rviz" "$HOME/.rviz2/default.rviz"
cat > "$rviz_application_dir/robot-navigation-rviz.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=Robot Navigation RViz
Comment=Map, laser scan, robot pose and native Nav2 path
Exec="$rviz_data_dir/start_rviz.sh"
Icon=rviz
Terminal=false
Categories=Science;Robotics;
EOF
install -m 0644 "$rviz_application_dir/robot-navigation-rviz.desktop" \
  "$rviz_config_dir/autostart/robot-navigation-rviz.desktop"
printf 'Installed RViz defaults and login autostart. Launch: %s\n' "$rviz_data_dir/start_rviz.sh"
