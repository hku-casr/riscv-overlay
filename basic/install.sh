#!/bin/bash

inputfile="macro_template.txt"
device="Xilinx"
legacy=false

preLine="\`ifndef macros_para
\`define macros_para
"

echo "Device from which company?"
select de in "Xilinx" "Altera"; do
    case $de in
        Xilinx ) device="Xilinx"; preLine="$preLine\`define XILINX
"; break;;
        Altera ) device="Altera"; preLine="$preLine//\`define XILINX
//\`define LEG_BOARD
"; break;;
    esac
done

echo $device "is chosen"

if [ $device == "Xilinx" ]; then
    echo "Is your device order than Virtex 5 series (V5 not included)?"
    select ys in "Yes" "No"; do
    case $ys in
        Yes ) legacy=true; preLine="$preLine\`define LEG_BOARD
"; break;;
        No ) legacy=false; preLine="$preLine//\`define LEG_BOARD
"; break;;
    esac
    done
fi

re='^[0-9]+$'
echo "How big is the IMEM in the power of 2?"
read imem_size
while true ;do
    if ! [[ $imem_size =~ $re ]] ; then
        echo "Please enter a positive integer";
        read imem_size;
    else
        break;
    fi
done

preLine="$preLine
\`define IMEMM_ADDR_BIT_NUM    $imem_size
"
imem_depth=`echo "2 ^ $imem_size" | bc`

preLine="$preLine\`define IMEMM_DEPTH    $imem_depth
"




echo "How big is the DMEM in the power of 2?"
read dmem_size
while true ;do
    if ! [[ $dmem_size =~ $re ]] ; then
        echo "Please enter a positive integer";
        read dmem_size;
    else
        break;
    fi
done

preLine="$preLine\`define DMEMM_ADDR_BIT_NUM    $dmem_size
"
dmem_depth=`echo "2 ^ $dmem_size" | bc`

preLine="$preLine\`define DMEMM_DEPTH    $dmem_depth
"


rm macro_para.v

echo "$preLine" >> macro_para.v

#echo the rest of the macro_para.v
while read -r line
do
    name=$line
    echo "$name" >> macro_para.v
done < "$inputfile"

echo "Done!"
