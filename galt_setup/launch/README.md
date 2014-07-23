# Writing launch files

While writing launch files for *Galt*, there are some guidelines to which you should adhere:

* **Hardware devices**: Prefix your launch file with the name of the package or device. Eg. `flir_320.launch`
* If necessary, suffix your launch files with a keyword which hints at the configuration they employ. Eg. `flir_320_fastmode.launch`
* **Preview launch files**: Launch files which trigger preview/demo tools (such as RQT) should inlude the argument `hard_launch`. If `hard_launch` is true, you must also launch the underlying device or algorithm - otherwise launch *only* the UI elments. See `bluefox2_stereo_preview` for an example.
* Do **not** include launch files from packages which fall outside of project Galt. These are in third-party repositories and may change substantially between versions. Write a full launcher using the `<node>` tag.
* Refrain from starting GUI objects by default, unless this is the explicit purpose of the launch file.
* Document your launch files with a descriptive header, and a list of topics which ought to be recorded by rosbag. For example:

```xml
<!--
  Name: flir_gige.launch
  Triggers:
    * fir_gige

  Brief: Launch the flir_gige node at 20fps.

  Topics you should record:
    * /flir_gige/image_raw
    * /flir_gige/camera_info
    * /flir_gige/spot
-->
<launch>
  <!-- ... launch stuff goes here -->
</launch>
```
