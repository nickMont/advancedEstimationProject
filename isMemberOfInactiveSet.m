function isMemberOfInactiveSetTrue = isMemberOfInactiveSet(index,inactivesetbox)

timesAppearingInInactiveSet=sum(ismember(index,inactivesetbox));
isMemberOfInactiveSetTrue=(timesAppearingInInactiveSet~=0);

end

