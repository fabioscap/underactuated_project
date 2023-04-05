sol = load('../build/SOLUTION.txt');
err = load('../build/ERRORS.txt');
plot(sol(:,end)); hold on;
plot(sol(:,end-6))
%plot(err)

i = 0;