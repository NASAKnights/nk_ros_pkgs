import sys
import docker
import subprocess
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QListWidget, QLineEdit, QTextEdit
from PyQt5.QtGui import QFont

CONTAINER_PREFIX = "ros_gui_"  # Define a global prefix

class DeployContainerPopup(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        self.setWindowTitle("Deploy New Container")
        self.setGeometry(150, 150, 500, 400)
        
        self.setStyleSheet("""
            QWidget {
                background-color: #2E2E2E;
                color: #FFFFFF;
                font-size: 14px;
            }
            QLabel {
                color: #FFFFFF;
                font-size: 14px;
            }
            QLineEdit, QTextEdit {
                background-color: #3E3E3E;
                color: #FFFFFF;
                border: 1px solid #777777;
                font-size: 14px;
            }
            QPushButton {
                background-color: #444444;
                color: white;
                padding: 8px;
                border-radius: 5px;
                border: 1px solid #777777;
                font-size: 14px;
            }
        """)
        
        layout = QVBoxLayout()

        layout.addWidget(QLabel("ROS Workspace Path:"))
        self.workspace_input = QLineEdit("/workspaces/isaac_ros-dev/ros_ws")
        layout.addWidget(self.workspace_input)

        layout.addWidget(QLabel("ROS 2 Launch Package:"))
        self.launch_package_input = QLineEdit("my_package")
        layout.addWidget(self.launch_package_input)

        layout.addWidget(QLabel("ROS 2 Launch File:"))
        self.launch_file_input = QLineEdit("my_launch.py")
        layout.addWidget(self.launch_file_input)

        self.deploy_button = QPushButton("Deploy Container")
        self.deploy_button.clicked.connect(self.deploy_container)
        layout.addWidget(self.deploy_button)

        self.terminal_output = QTextEdit()
        self.terminal_output.setReadOnly(True)
        layout.addWidget(QLabel("Deployment Output:"))
        layout.addWidget(self.terminal_output)

        self.setLayout(layout)
    
    def deploy_container(self):
        ros_workspace = self.workspace_input.text().strip()
        launch_package = self.launch_package_input.text().strip()
        launch_file = self.launch_file_input.text().strip()
        container_name = f"{CONTAINER_PREFIX}{launch_file.replace('.launch.py', '').replace('.launch', '')}"

        deploy_command = f"bash $ISAAC_ROS_WS/src/isaac_ros_common/scripts/docker_deploy.sh -n {container_name} -w {ros_workspace} -p {launch_package} -f {launch_file}"
        process = subprocess.Popen(deploy_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        output, error = process.communicate()
        self.terminal_output.setText(output + error)
        
        self.parent.refresh_container_list()
        self.parent.refresh_image_list()

class DockerManagerGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.docker_client = docker.from_env()
        self.setStyleSheet("""
            QWidget {
                background-color: #2E2E2E;
                color: #FFFFFF;
                font-size: 14px;
            }
            QLabel {
                color: #FFFFFF;
                font-size: 14px;
            }
            QPushButton {
                background-color: #444444;
                color: white;
                padding: 8px;
                border-radius: 5px;
                border: 1px solid #777777;
                font-size: 14px;
            }
        """)
        self.initUI()

    def initUI(self):
        self.setWindowTitle("Isaac ROS Docker Manager")
        self.setGeometry(100, 100, 500, 600)
        
        layout = QVBoxLayout()

        self.add_deployment_button = QPushButton("Add New Deployment")
        self.add_deployment_button.clicked.connect(self.open_deployment_popup)
        layout.addWidget(self.add_deployment_button)

        self.image_list = QListWidget()
        self.refresh_image_list()
        layout.addWidget(QLabel("Available Images:"))
        layout.addWidget(self.image_list)

        self.start_image_button = QPushButton("Start Selected Image")
        self.start_image_button.clicked.connect(self.start_image)
        layout.addWidget(self.start_image_button)

        self.container_list = QListWidget()
        self.refresh_container_list()
        layout.addWidget(QLabel("Running Containers:"))
        layout.addWidget(self.container_list)
        
        self.stop_button = QPushButton("Stop Selected Container")
        self.stop_button.clicked.connect(self.stop_container)
        layout.addWidget(self.stop_button)

        self.refresh_button = QPushButton("Refresh Containers")
        self.refresh_button.clicked.connect(self.refresh_container_list)
        layout.addWidget(self.refresh_button)
        
        self.setLayout(layout)

    def open_deployment_popup(self):
        self.deploy_popup = DeployContainerPopup(self)
        self.deploy_popup.show()

    def refresh_container_list(self):
        self.container_list.clear()
        containers = self.docker_client.containers.list(all=True)
        managed_containers = [c for c in containers if c.name.startswith(CONTAINER_PREFIX)]

        if not managed_containers:
            self.container_list.addItem("No managed containers found.")

        for container in managed_containers:
            display_name = container.name[len(CONTAINER_PREFIX):]
            image = container.image.tags[0] if container.image.tags else "Unknown Image"
            status = container.status.upper()
            command = " ".join(container.attrs["Config"]["Cmd"]) if container.attrs["Config"]["Cmd"] else "N/A"
            short_id = container.short_id
            display_text = f"""
─────────────────────────────────────────
📦  **{display_name}**  ({short_id})
🔹 Image: `{image}`
🔹 Status: **{status}**
🔹 Command: `{command}`
─────────────────────────────────────────
"""
            self.container_list.addItem(display_text.strip())

    def refresh_image_list(self):
        self.image_list.clear()
        images = [img for img in self.docker_client.images.list() if any(tag.startswith(CONTAINER_PREFIX) for tag in img.tags)]
        if not images:
            self.image_list.addItem("No managed images found.")

        for image in images:
            image_tags = image.tags[0] if image.tags else "<untagged>"
            image_id = image.short_id
            self.image_list.addItem(f"🖼  {image_tags} ({image_id})")

    def start_selected_image(self):
        selected_items = self.image_list.selectedItems()
        if not selected_items:
            return
        image_name = selected_items[0].text().split()[0]
        container_name = f"{CONTAINER_PREFIX}{image_name.split(':')[0]}"
        self.docker_client.containers.run(
            image=image_name,
            name=container_name,
            detach=True,
            restart_policy={"Name": "always"},
            network="host"
        )
        self.refresh_container_list()

    def stop_container(self):
        selected_items = self.container_list.selectedItems()
        if not selected_items:
            return
        display_name = selected_items[0].text().split(" ")[1]
        container_name = f"{CONTAINER_PREFIX}{display_name}"
        container = self.docker_client.containers.get(container_name)
        container.stop()
        self.refresh_container_list()
    
    def start_image(self):
        """ Start a container from a selected image with restart policy. """
        selected_items = self.image_list.selectedItems()
        if not selected_items:
            return

        image_tag = selected_items[0].text().split(" ")[1]  # Extract image tag
        container_name = f"{CONTAINER_PREFIX}{image_tag.split(':')[0]}"

        self.docker_client.containers.run(
            image=image_tag,
            name=container_name,
            detach=True,
            restart_policy={"Name": "always"},
            network="host"
        )

        self.refresh_container_list()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DockerManagerGUI()
    window.show()
    sys.exit(app.exec())