#!/bin/sh

echo "=================== make template ==================="
python template.py  -s data/partsA.ght -r 242 507 458 414 data/template_full.ply

echo ""

echo "================= detection execute ================="
echo "-------------------- detection 2D -------------------"
python detector.py -l=data/partsA.ght -r 317 284 490 574 data/detection_full.ply 

echo ""

echo "-------------------- detection 3D -------------------"
python detector.py -l=data/partsA.ght -r 366 363 350 478 data/detection_full.ply  -t data/model.ply  -i 


