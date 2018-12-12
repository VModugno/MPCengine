function  [H,F_tra]=costFunction_regulator_std(S_bar,T_bar,Q_bar,R_bar)

    H = 2*(R_bar + S_bar'*Q_bar*S_bar);
    F_tra = 2*T_bar'*Q_bar*S_bar;
end