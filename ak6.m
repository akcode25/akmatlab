% bank account

b = 300000; w = 25000; r = 0.05; inf = 0.02; y = 0;
while b > 0
    y = y + 1; 
    b = b * (1 + r) - w; % Update balance after interest & withdrawal
    B(y) = b;            % Store balance
    W(y) = w;            % Store withdrawal
    w = w * (1 + inf);   % Adjust withdrawal for inflation
end
plot(1:y, W, 'r', 1:y, [300000 B(1:end-1)], 'b');    % withdrawals % balance
xlabel('Year'); ylabel('Amount ($)'); grid on;
legend('Withdrawals', 'Balance'); title('Withdrawals and Balance');