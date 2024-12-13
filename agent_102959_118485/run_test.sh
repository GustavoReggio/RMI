#!/bin/bash

challenge="4"
host="localhost"
robname="theAgent"
pos="0"
outfile="solution"

while getopts "c:h:r:p:f:" op
do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "h")
            host=$OPTARG
            ;;
        "r")
            robname=$OPTARG
            ;;
        "p")
            pos=$OPTARG
            ;;
        "f")
            outfile=$OPTARG
            ;;
        default)
            echo "ERROR in parameters"
            ;;
    esac
done

shift $(($OPTIND-1))

case $challenge in
    4)
        # how to call agent for challenge 4
        ./mainC4.py -h "$host" -p "$pos" -r 1 -f "$outfile" &
        ./mainC4.py -h "$host" -p "$pos" -r 2 -f "$outfile" &
        ./mainC4.py -h "$host" -p "$pos" -r 3 -f "$outfile" &
        ./mainC4.py -h "$host" -p "$pos" -r 4 -f "$outfile" &
        ./mainC4.py -h "$host" -p "$pos" -r 5 -f "$outfile" &

        wait
        # mv map_created.map $outfile.map            # if needed
        # mv path_created.path $outfile.path           # if needed
esac
"$@"