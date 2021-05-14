#!/bin/bash

# Script de instalação da biblioteca DD no sistema operacional nativo
KVERSION=4.7.4
KERNEL_PATH=/home/dimitri/Projetos/einstein_a20/kernels/linux-4.7/output/lib/modules/$KVERSION/kernel/drivers
TARGET_DIR=/lib/modules/$KVERSION/kernel/drivers/misc/

# Função para instalar na pasta output do kernel
function do_install() {
	local filename=$(basename "$1")
	local filepath=$KERNEL_PATH/misc
	mkdir -p $filepath
	echo "copy $filename to $filepath"
	cp $filename $filepath 
}

# Função da instalação da biblicdoteca no embarcado
function do_install_embedded() {
    local libfile=$(readlink -f "$1")
	local filename=$(basename "$1")
	local address=$2
    
    # Copiar a biblioteca para o equipamento
    scp -p $libfile root@$address:$TARGET_DIR
    ssh root@$address "depmod -a"
}

function die() {
	echo "$*" >&2
	exit 1
}

function show_usage_and_die() {
	echo "Usage (install on embedded): $0 -e [input.so.x.x.x] ip-address"
	exit 1
}

# [ $# -eq 1 ] || show_usage_and_die

getopts l:e: opt;
	case "$opt" in
		l) do_install $2 ;;
		e) do_install_embedded $2 $3 ;;
		:) show_usage_and_die ;;
		*) show_usage_and_die ;;
	esac

# do_install $1
