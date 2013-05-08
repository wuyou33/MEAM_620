%Copyright KMel Robotics 2012. Must read KMEL_LICENSE.pdf for terms and conditions before use.
%decide if it is time to switch sequences
endCond

%compute the control based on the current mode
if(seqM(qn).seq(seq_cntM(qn)).type==700)
    directMode
elseif(seqM(qn).seq(seq_cntM(qn)).type==55 || seqM(qn).seq(seq_cntM(qn)).type==755 || seqM(qn).seq(seq_cntM(qn)).type==7)
    setGains
    
    if(seqM(qn).seq(seq_cntM(qn)).type==55)
        waypointMode
    elseif(seqM(qn).seq(seq_cntM(qn)).type==755)
        takeoffMode
    elseif(seqM(qn).seq(seq_cntM(qn)).type==7)
        hoverMode
    end
    
    positionControl
elseif(seqM(qn).seq(seq_cntM(qn)).type==901)
    student_control_hover
elseif(seqM(qn).seq(seq_cntM(qn)).type==902)
    student_control_waypt    
elseif(seqM(qn).seq(seq_cntM(qn)).type==903)
    student_control_multi_waypt 
else
    hoverMode
    positionControl
end

%apply the safety logic
safetyLogic

%send out the command
sendCmd