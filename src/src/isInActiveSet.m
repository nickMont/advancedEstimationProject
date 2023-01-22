function isMemberOfInactiveSetTrue = isMemberOfInactiveSet(index,activesetbox)

timesAppearingInActiveSet=sum(ismember(index,activesetbox));
isMemberOfInactiveSetTrue=(timesAppearingInActiveSet==0);

end

