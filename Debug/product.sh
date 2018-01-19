#!/bin/bash

read -p "Ввести спартовый номер (1..200): " START

for(( i=${START}; i < 201; i= $i+1 )); do 
	j=$(sed -n "${i}p" ../uid.list)
	let d1=$(echo $j| cut -d" " -f 1)+10
	d2=$(echo $j| cut -d" " -f 2); 

	PATH="/home/jet/work/tools/toolchain/gcc-arm-none-eabi-6_2-2016q4/bin:${PATH}"
	CFLAGS="-DBDADDR=${d1} -DDEVICE_UUID=${d2} ${CFLAGS}"
	read -p  "Цикл $i: BDADDR=$d1, DEVICE_UUID=$d2. <Enter> для кампиляции:" e

	echo "#define  BDADDR  ${d1}" > ../include/deviceid.h
	echo "#define  DEVICE_UUID  \"${d2}\"" >> ../include/deviceid.h
	make clean  && make all

	if [ $? -ne 0 ]; then
		echo "Ошибка компиляции - выходим !"
		exit
	else
		echo "Успешная компиляция. Готово к прошивке."
		read -p "Как все будет готово - нажмите <Enter>" e
	fi

	for(( ;; )); do
		st-flash erase && st-flash write ./bt.relay.bin 0x08000000 && st-flash read ./1 0x08000000 14736

		if [ $(md5sum ./bt.relay.bin|cut -d" " -f1) = $(md5sum ./1|cut -d" " -f1) ]; then
			echo "========== Записано верно ! ================"
			read -p "Для продолжения нажмите <Enter>: "
			break;
		else
			echo " !!!!!!!!! ОШИБКА ЗАПИСИ !!!!!!!!!!!!!!!!!!!"
			read -p " Цикл $i.   Для повтора записи нажмите <Enter>, для выхода из программы - <CTRL>C: " e
		fi
	done
done
