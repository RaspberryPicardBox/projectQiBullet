import os

default_text1 = """\"<robot name=\"_name\">
	<link name=\"_name\">
		<visual>
			<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
			<geometry>
				<mesh filename=\""""

default_text2 = """\" scale=\"1.0 1.0 1.0\" />
			</geometry>
			<material name=\"texture\">
				<color rgba=\"1.0 1.0 1.0 1.0\"/>
			</material>
		</visual>
		<collision>
			<geometry>
				<mesh filename=\""""
default_text3 = """\" scale=\"1.0 1.0 1.0\" />
			</geometry>
		</collision>
		<inertial>
			<mass value=\"1.0\"/>
			<inertia ixx=\"1.0\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"1.0\" iyz=\"0.0\" izz=\"1.0\"/>
		</inertial>
	</link>
</robot>"""


def __init__():
    for foldername in os.listdir("./objects"):
        if foldername[-5:].lower() == ".urdf":
            os.remove("./objects/" + foldername)

    for foldername in os.listdir("./objects"):
        if foldername[-5:].lower() == ".urdf":
            os.remove("./objects/" + foldername)
            break
        for filename in os.listdir("./objects/" + foldername):
            if (filename[-4:].lower() == ".stl") or (filename[-4:].lower() == ".obj"):
                urdf = "./objects/"
                urdf += filename[:-4]
                urdf += ".urdf"
                if not os.path.isfile(urdf):
                    f = open(urdf, "w")
                    f.write(default_text1)
                    f.write("{0}\{1}".format(foldername, filename))
                    f.write(default_text2)
                    f.write("{0}\{1}".format(foldername, filename))
                    f.write(default_text3)
                    f.close()
