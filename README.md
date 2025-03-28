# nrf5340dk_max77643_driver
this is the driver of nrf5340dk connecting max77643
The steps of configuration:
1. put max77643.yaml under your ncs/your_version/zephyr/dts/bindings/regulator
2. using the overlay file in boards folder
3. modify your own contents of  CMakeLists.txt to the CMakLists.txt in this project
4. add max77643_2_regs.h/max77643.c/max77643.h to your projects/src
5. using prj.conf and nrf5340dk_nrf5340_cpuapp to build configuration

I think this driver can be both realize on max77643 and max77642.

Thanks for this website https://os.mbed.com/search/?q=max77643#gsc.tab=0&gsc.q=max77643&gsc.page=1 which helps me finish the code.
