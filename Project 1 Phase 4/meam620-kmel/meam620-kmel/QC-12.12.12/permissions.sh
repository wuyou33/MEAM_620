#!/bin/bash
find matlab/control -type f -exec chmod g-r '{}' \;
matlab -nodesktop <<EOF 
pcode -inplace matlab/control/limitForce.m
pcode -inplace matlab/control/positionControl.m  
pcode -inplace matlab/control/setGains.m
pcode -inplace matlab/control/updateIntegral.m
pcode -inplace matlab/control/controllers/*
EOF
echo ""

sudo chown -R katy:meam620 .
find matlab/control -name "*.p" -exec chmod g+r '{}' \;
for file in DoQuadControl.m endCond.m sendCmd.m safetyLogic.m; do
    chmod g+r matlab/control/$file
done

chmod g+r matlab/control
