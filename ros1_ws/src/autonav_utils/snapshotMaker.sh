#!/bin/bash

while [ "$option" != "q" ]
do 
	echo""
	echo "Welcome to Snapshot Maker"

	echo "Enter desired mode: a(dd), r(emove), l(ist), f(ile size), re(cover), or q(uit)"
	read option


	if [ "$option" = "a" ];
	then
		echo "enter snapshot name"
		read title
		ssh higgins-kortje@jabez mkdir /home/higgins-kortje/robotBackup/$title 
		scp -r /home/autonav/ higgins-kortje@jabez:/home/higgins-kortje/robotBackup/$title &
		ssh higgins-kortje@jabez rm /home/higgins-kortje/robotBackup/$title/autonav/Documents/*.bag

	elif [ "$option" = "r" ];
	then
		echo "current snapshots:"
		ssh higgins-kortje@jabez ls /home/higgins-kortje/robotBackup
		echo "select a snapshot:"
		read snapshot
		ssh higgins-kortje@jabez rm -r -f /home/higgins-kortje/robotBackup/$snapshot

	elif [ "$option" = "re" ];
	then
		echo "current snapshots"
			ssh higgins-kortje@jabez ls /home/higgins-kortje/robotBackup
			echo "select a snapshot:"
			read snapshot
			scp -r higgins-kortje@jabez:/home/higgins-kortje/robotBackup/$snapshot/ /home/

	elif [ "$option" = "f" ];
	then
		echo ""
			echo "current snapshots"
		ssh higgins-kortje@jabez ls /home/higgins-kortje/robotBackup
			echo "select a snapshot:"
			read snapshot
			ssh higgins-kortje@jabez du -h /home/higgins-kortje/robotBackup/$snapshot
		echo ""


	elif [ "$option" = "l" ];
	then
		echo "current snapshots:"
			ssh higgins-kortje@jabez ls /home/higgins-kortje/robotBackup
	
	elif [ "$option" = "q" ];
	then
		echo "closing..."


	else
		echo "Invalid Input"
	fi

	echo ""
done
