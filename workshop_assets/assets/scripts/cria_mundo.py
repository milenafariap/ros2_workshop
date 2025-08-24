def gerar_sdf_mundo(
    num_tomates: int = 30,
    linhas: int = 3,
    colunas: int = 20,
    espacamento: float = 1,
    output_path: str = "farm_world.sdf"
):
    tomato_model_uri = "https://fuel.gazebosim.org/1.0/omermaayany/models/naked tomato plant"
    
    # Cabeçalho do SDF
    sdf = f"""<sdf version='1.9'>
  <world name='farm_world'>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.6 0.8 0.6 1</background>
      <shadows>true</shadows>
    </scene>
    <gravity>0 0 -9.8</gravity>

    <!-- Plano de chão verde simples -->
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>50 50</size>
            </plane>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>50 50</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.1 0.4 0.1 1</ambient>
            <diffuse>0.1 0.6 0.1 1</diffuse>
            <specular>0 0 0 1</specular>
          </material>
        </visual>
      </link>
    </model>

        <model name='vehicle_blue' canonical_link='base_link'>
        <pose>-3 0 0 0 -0 0</pose>
        <frame name='lidar_frame' attached_to='base_link'>
            <pose>0.1 0 0.2 0 0 0</pose>
        </frame>
        <link name='base_link'>
            <pose>0.5 0 0.2 0 -0 0</pose>
            <inertial>
            <mass>1.14395</mass>
            <inertia>
                <ixx>0.095328999999999997</ixx>
                <ixy>0</ixy>
                <ixz>0</ixz>
                <iyy>0.38131700000000002</iyy>
                <iyz>0</iyz>
                <izz>0.47664600000000001</izz>
            </inertia>
            <pose>0 0 0 0 -0 0</pose>
            </inertial>
            <visual name='visual'>
            <geometry>
                <box>
                <size>0.5 0.2 0.1</size>
                </box>
            </geometry>
            <material>
                <ambient>0 0 1 1</ambient>
                <diffuse>0 0 1 1</diffuse>
                <specular>0 0 1 1</specular>
            </material>
            </visual>
            <collision name='collision'>
            <geometry>
                <box>
                <size>0.5 0.2 0.1</size>
                </box>
            </geometry>
            <surface>
                <friction>
                <ode/>
                </friction>
                <bounce/>
                <contact/>
            </surface>
            </collision>
            <sensor name='imu_sensor' type='imu'>
            <pose>0 0 0 0 -0 0</pose>
            <topic>imu</topic>
            <update_rate>1</update_rate>
            <enable_metrics>false</enable_metrics>
            <imu>
                <orientation_reference_frame>
                <localization>CUSTOM</localization>
                <custom_rpy>0 0 0</custom_rpy>
                <grav_dir_x>1 0 0</grav_dir_x>
                </orientation_reference_frame>
                <angular_velocity>
                <x>
                    <noise type='none'>
                    <mean>0</mean>
                    <stddev>0</stddev>
                    <bias_mean>0</bias_mean>
                    <bias_stddev>0</bias_stddev>
                    <dynamic_bias_stddev>0</dynamic_bias_stddev>
                    <dynamic_bias_correlation_time>0</dynamic_bias_correlation_time>
                    <precision>0</precision>
                    </noise>
                </x>
                <y>
                    <noise type='none'>
                    <mean>0</mean>
                    <stddev>0</stddev>
                    <bias_mean>0</bias_mean>
                    <bias_stddev>0</bias_stddev>
                    <dynamic_bias_stddev>0</dynamic_bias_stddev>
                    <dynamic_bias_correlation_time>0</dynamic_bias_correlation_time>
                    <precision>0</precision>
                    </noise>
                </y>
                <z>
                    <noise type='none'>
                    <mean>0</mean>
                    <stddev>0</stddev>
                    <bias_mean>0</bias_mean>
                    <bias_stddev>0</bias_stddev>
                    <dynamic_bias_stddev>0</dynamic_bias_stddev>
                    <dynamic_bias_correlation_time>0</dynamic_bias_correlation_time>
                    <precision>0</precision>
                    </noise>
                </z>
                </angular_velocity>
                <linear_acceleration>
                <x>
                    <noise type='none'>
                    <mean>0</mean>
                    <stddev>0</stddev>
                    <bias_mean>0</bias_mean>
                    <bias_stddev>0</bias_stddev>
                    <dynamic_bias_stddev>0</dynamic_bias_stddev>
                    <dynamic_bias_correlation_time>0</dynamic_bias_correlation_time>
                    <precision>0</precision>
                    </noise>
                </x>
                <y>
                    <noise type='none'>
                    <mean>0</mean>
                    <stddev>0</stddev>
                    <bias_mean>0</bias_mean>
                    <bias_stddev>0</bias_stddev>
                    <dynamic_bias_stddev>0</dynamic_bias_stddev>
                    <dynamic_bias_correlation_time>0</dynamic_bias_correlation_time>
                    <precision>0</precision>
                    </noise>
                </y>
                <z>
                    <noise type='none'>
                    <mean>0</mean>
                    <stddev>0</stddev>
                    <bias_mean>0</bias_mean>
                    <bias_stddev>0</bias_stddev>
                    <dynamic_bias_stddev>0</dynamic_bias_stddev>
                    <dynamic_bias_correlation_time>0</dynamic_bias_correlation_time>
                    <precision>0</precision>
                    </noise>
                </z>
                </linear_acceleration>
                <enable_orientation>true</enable_orientation>
            </imu>
            </sensor>
            <sensor name='gpu_lidar' type='gpu_lidar'>
            <pose>0.8 0 0.5 0 -0 0</pose>
            <topic>lidar</topic>
            <update_rate>10</update_rate>
            <enable_metrics>false</enable_metrics>
            <lidar>
                <scan>
                <horizontal>
                    <samples>640</samples>
                    <resolution>1</resolution>
                    <min_angle>-1.3962600000000001</min_angle>
                    <max_angle>1.3962600000000001</max_angle>
                </horizontal>
                <vertical>
                    <samples>1</samples>
                    <min_angle>0</min_angle>
                    <max_angle>0</max_angle>
                    <resolution>0.01</resolution>
                </vertical>
                </scan>
                <range>
                <min>0.080000000000000002</min>
                <max>10</max>
                <resolution>0.01</resolution>
                </range>
                <noise>
                <type>none</type>
                <mean>0</mean>
                <stddev>0</stddev>
                </noise>
                <visibility_mask>4294967295</visibility_mask>
            </lidar>
            </sensor>
            <enable_wind>false</enable_wind>
        </link>
        <link name='camera_link'>
            <pose>0.5 0 0.4 0 -0 0</pose>
            <inertial>
            <mass>0.10000000000000001</mass>
            <inertia>
                <ixx>0.00016666700000000001</ixx>
                <iyy>0.00016666700000000001</iyy>
                <izz>0.00016666700000000001</izz>
                <ixy>0</ixy>
                <ixz>0</ixz>
                <iyz>0</iyz>
            </inertia>
            <pose>0 0 0 0 -0 0</pose>
            </inertial>
            <collision name='collision'>
            <geometry>
                <box>
                <size>0.1 0.1 0.1</size>
                </box>
            </geometry>
            <surface>
                <friction>
                <ode/>
                </friction>
                <bounce/>
                <contact/>
            </surface>
            </collision>
            <visual name='visual'>
            <geometry>
                <box>
                <size>0.1 0.1 0.1</size>
                </box>
            </geometry>
            </visual>
            <sensor name='camera' type='camera'>
            <pose>0 0 0 0 -0 0</pose>
            <topic>camera</topic>
            <update_rate>30</update_rate>
            <enable_metrics>false</enable_metrics>
            <camera name='__default__'>
                <pose>0 0 0 0 -0 0</pose>
                <horizontal_fov>1.0469999999999999</horizontal_fov>
                <image>
                <width>380</width>
                <height>240</height>
                <format>RGB_INT8</format>
                <anti_aliasing>4</anti_aliasing>
                </image>
                <camera_info_topic>__default__</camera_info_topic>
                <trigger_topic></trigger_topic>
                <triggered>false</triggered>
                <clip>
                <near>0.10000000000000001</near>
                <far>100</far>
                </clip>
                <save enabled='false'>
                <path>__default__</path>
                </save>
                <visibility_mask>4294967295</visibility_mask>
                <noise>
                <type>none</type>
                <mean>0</mean>
                <stddev>0</stddev>
                </noise>
                <distortion>
                <k1>0</k1>
                <k2>0</k2>
                <k3>0</k3>
                <p1>0</p1>
                <p2>0</p2>
                <center>0.5 0.5</center>
                </distortion>
                <lens>
                <type>stereographic</type>
                <scale_to_hfov>true</scale_to_hfov>
                <cutoff_angle>1.5708</cutoff_angle>
                <env_texture_size>256</env_texture_size>
                </lens>
                <optical_frame_id></optical_frame_id>
            </camera>
            </sensor>
            <enable_wind>false</enable_wind>
        </link>
        <joint name='camera' type='fixed'>
            <parent>base_link</parent>
            <child>camera_link</child>
            <pose>0 0 0 0 -0 0</pose>
        </joint>
        <link name='left_wheel1'>
            <pose>0.4 0.12 0.15 -1.5707 0 0</pose>
            <inertial>
            <mass>1</mass>
            <inertia>
                <ixx>0.043333000000000003</ixx>
                <ixy>0</ixy>
                <ixz>0</ixz>
                <iyy>0.043333000000000003</iyy>
                <iyz>0</iyz>
                <izz>0.080000000000000002</izz>
            </inertia>
            <pose>0 0 0 0 -0 0</pose>
            </inertial>
            <visual name='visual'>
            <geometry>
                <cylinder>
                <radius>0.07</radius>
                <length>0.10000000000000001</length>
                </cylinder>
            </geometry>
            <material>
                <ambient>1 0 0 1</ambient>
                <diffuse>1 0 0 1</diffuse>
                <specular>1 0 0 1</specular>
            </material>
            </visual>
            <collision name='collision'>
            <geometry>
                <cylinder>
                <radius>0.07</radius>
                <length>0.10000000000000001</length>
                </cylinder>
            </geometry>
            <surface>
                <friction>
                <ode/>
                </friction>
                <bounce/>
                <contact/>
            </surface>
            </collision>
            <enable_wind>false</enable_wind>
        </link>
        <link name='right_wheel1'>
            <pose>0.4 -0.12 0.15 -1.5707 0 0</pose>
            <inertial>
            <mass>1</mass>
            <inertia>
                <ixx>0.043333000000000003</ixx>
                <ixy>0</ixy>
                <ixz>0</ixz>
                <iyy>0.043333000000000003</iyy>
                <iyz>0</iyz>
                <izz>0.080000000000000002</izz>
            </inertia>
            <pose>0 0 0 0 -0 0</pose>
            </inertial>
            <visual name='visual'>
            <geometry>
                <cylinder>
                <radius>0.07</radius>
                <length>0.10000000000000001</length>
                </cylinder>
            </geometry>
            <material>
                <ambient>1 0 0 1</ambient>
                <diffuse>1 0 0 1</diffuse>
                <specular>1 0 0 1</specular>
            </material>
            </visual>
            <collision name='collision'>
            <geometry>
                <cylinder>
                <radius>0.07</radius>
                <length>0.10000000000000001</length>
                </cylinder>
            </geometry>
            <surface>
                <friction>
                <ode/>
                </friction>
                <bounce/>
                <contact/>
            </surface>
            </collision>
            <enable_wind>false</enable_wind>
        </link>
        <link name='right_wheel2'>
            <pose>0.7 -0.12 0.15 -1.5707 0 0</pose>
            <inertial>
            <mass>1</mass>
            <inertia>
                <ixx>0.043333000000000003</ixx>
                <ixy>0</ixy>
                <ixz>0</ixz>
                <iyy>0.043333000000000003</iyy>
                <iyz>0</iyz>
                <izz>0.080000000000000002</izz>
            </inertia>
            <pose>0 0 0 0 -0 0</pose>
            </inertial>
            <visual name='visual'>
            <geometry>
                <cylinder>
                <radius>0.07</radius>
                <length>0.10000000000000001</length>
                </cylinder>
            </geometry>
            <material>
                <ambient>1 0 0 1</ambient>
                <diffuse>1 0 0 1</diffuse>
                <specular>1 0 0 1</specular>
            </material>
            </visual>
            <collision name='collision'>
            <geometry>
                <cylinder>
                <radius>0.07</radius>
                <length>0.10000000000000001</length>
                </cylinder>
            </geometry>
            <surface>
                <friction>
                <ode/>
                </friction>
                <bounce/>
                <contact/>
            </surface>
            </collision>
            <enable_wind>false</enable_wind>
        </link>
        <link name='left_wheel2'>
            <pose>0.7 0.12 0.15 -1.5707 0 0</pose>
            <inertial>
            <mass>1</mass>
            <inertia>
                <ixx>0.043333000000000003</ixx>
                <ixy>0</ixy>
                <ixz>0</ixz>
                <iyy>0.043333000000000003</iyy>
                <iyz>0</iyz>
                <izz>0.080000000000000002</izz>
            </inertia>
            <pose>0 0 0 0 -0 0</pose>
            </inertial>
            <visual name='visual'>
            <geometry>
                <cylinder>
                <radius>0.07</radius>
                <length>0.10000000000000001</length>
                </cylinder>
            </geometry>
            <material>
                <ambient>1 0 0 1</ambient>
                <diffuse>1 0 0 1</diffuse>
                <specular>1 0 0 1</specular>
            </material>
            </visual>
            <collision name='collision'>
            <geometry>
                <cylinder>
                <radius>0.07</radius>
                <length>0.10000000000000001</length>
                </cylinder>
            </geometry>
            <surface>
                <friction>
                <ode/>
                </friction>
                <bounce/>
                <contact/>
            </surface>
            </collision>
            <enable_wind>false</enable_wind>
        </link>
        <joint name='left_wheel1_joint' type='revolute'>
            <pose>0 0 -0 0 -0 0</pose>
            <parent>base_link</parent>
            <child>left_wheel1</child>
            <axis>
            <xyz>0 9.6e-05 1</xyz>
            <limit>
                <lower>-1.7976900000000001e+308</lower>
                <upper>1.7976900000000001e+308</upper>
                <effort>inf</effort>
                <velocity>inf</velocity>
                <stiffness>100000000</stiffness>
                <dissipation>1</dissipation>
            </limit>
            <dynamics>
                <spring_reference>0</spring_reference>
                <spring_stiffness>0</spring_stiffness>
                <damping>0</damping>
                <friction>0</friction>
            </dynamics>
            </axis>
        </joint>
        <joint name='left_wheel2_joint' type='revolute'>
            <pose>0 0 -0 0 -0 0</pose>
            <parent>base_link</parent>
            <child>left_wheel2</child>
            <axis>
            <xyz>0 9.6e-05 1</xyz>
            <limit>
                <lower>-1.7976900000000001e+308</lower>
                <upper>1.7976900000000001e+308</upper>
                <effort>inf</effort>
                <velocity>inf</velocity>
                <stiffness>100000000</stiffness>
                <dissipation>1</dissipation>
            </limit>
            <dynamics>
                <spring_reference>0</spring_reference>
                <spring_stiffness>0</spring_stiffness>
                <damping>0</damping>
                <friction>0</friction>
            </dynamics>
            </axis>
        </joint>
        <joint name='right_wheel1_joint' type='revolute'>
            <pose>0 -0 -0 0 -0 0</pose>
            <parent>base_link</parent>
            <child>right_wheel1</child>
            <axis>
            <xyz>0 9.6e-05 1</xyz>
            <limit>
                <lower>-1.7976900000000001e+308</lower>
                <upper>1.7976900000000001e+308</upper>
                <effort>inf</effort>
                <velocity>inf</velocity>
                <stiffness>100000000</stiffness>
                <dissipation>1</dissipation>
            </limit>
            <dynamics>
                <spring_reference>0</spring_reference>
                <spring_stiffness>0</spring_stiffness>
                <damping>0</damping>
                <friction>0</friction>
            </dynamics>
            </axis>
        </joint>
        <joint name='right_wheel2_joint' type='revolute'>
            <pose>0 -0 -0 0 -0 0</pose>
            <parent>base_link</parent>
            <child>right_wheel2</child>
            <axis>
            <xyz>0 9.6e-05 1</xyz>
            <limit>
                <lower>-1.7976900000000001e+308</lower>
                <upper>1.7976900000000001e+308</upper>
                <effort>inf</effort>
                <velocity>inf</velocity>
                <stiffness>100000000</stiffness>
                <dissipation>1</dissipation>
            </limit>
            <dynamics>
                <spring_reference>0</spring_reference>
                <spring_stiffness>0</spring_stiffness>
                <damping>0</damping>
                <friction>0</friction>
            </dynamics>
            </axis>
        </joint>
        <plugin name='ignition::gazebo::systems::DiffDrive' filename='libignition-gazebo-diff-drive-system.so'>
            <left_joint>left_wheel1_joint</left_joint>
            <left_joint>left_wheel2_joint</left_joint>
            <right_joint>right_wheel1_joint</right_joint>
            <right_joint>right_wheel2_joint</right_joint>
            <wheel_separation>0.92</wheel_separation>
            <wheel_radius>0.07</wheel_radius>
            <odom_publish_frequency>30</odom_publish_frequency>
            <topic>cmd_vel</topic>
            <odom_topic>odom</odom_topic>
            <odom_frame>odom</odom_frame>
            <base_link>base_link</base_link>
            <publish_tf>true</publish_tf>
        </plugin>
        <pose>0 -2.02873 0 0 -0 0</pose>
        <static>false</static>
        <self_collide>false</self_collide>
        </model>
    """

    # Adiciona pés de tomate organizados em grade
    for i in range(num_tomates):
        row = (i // colunas) + 0.5
        col = (i % colunas) + 0.5
        x = col * espacamento
        y = row * espacamento
        sdf += f"""
    <include>
      <uri>{tomato_model_uri}</uri>
      <name>naked_tomato_{i}</name>
      <pose>{x} {y} 0 0 0 0</pose>
      <scale>1.0 1.0 1.0</scale>  <!-- Aumenta o tamanho em 50% -->
    </include>
"""

    # Luz do sol
    sdf += """
    <light name='sun' type='directional'>
      <pose>0 0 10 0 0 0</pose>
      <direction>-0.5 0.1 -1</direction>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <cast_shadows>true</cast_shadows>
    </light>
  </world>
</sdf>
"""

    with open(output_path, "w") as f:
        f.write(sdf)

    print(f"✅ Mundo salvo em: {output_path}")


# Exemplo de uso
if __name__ == "__main__":
    gerar_sdf_mundo(num_tomates=50, linhas=10, colunas=10)