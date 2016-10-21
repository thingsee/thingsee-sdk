The server side needs to have 2 files:

1) version.txt
2) nuttx.oci

The app reads a file "version.txt" from the server and scans the content for:

Version: <integer>

If the previously received & flashed firmware was tagged with a version number
smaller than the one available on the server, the app will download the binary
firmware file called "nuttx.oci" and restart the device.

The bootloader will then flash the firmware while booting up the next time.

Start fota automatically at boot:

1) install genromfs to host system:
        $ sudo apt install genromfs

2) add fota to rcS.template
        $ cd nuttx/configs/haltian-tsone/include
        $ editor_of_your_choice rcS.template

3) generate romfsimg:
        $ ../../../tools/mkromfsimg.sh ../../../

4) compile and flash
