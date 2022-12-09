function dxdt = InvertedPendulum(t,x,u,A,B)
    dxdt = A*x + B*u;
end
