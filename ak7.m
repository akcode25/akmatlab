% singers

singers = {'John', 'Mary', 'Tracy', 'Mike', 'Katie', 'David'};
randomOrder = randperm(length(singers));
disp('The random performance order is:');
for i = 1:length(singers)
   disp([num2str(i),'. ' singers{randomOrder(i)}]);
end

% disp(singers(randperm(length(singers))));