function H = computeH(order, m, mu_r, k_r, t)

polynomial_r = ones(1,order+1);
for i=1:k_r
    polynomial_r = polyder(polynomial_r);       %Differentiation
end

H = [];
for i=1:m
    A_y = zeros(order+1,order+1);
    A_z = zeros(order+1,order+1);
    
    for j=1:order+1
        for k=j:order+1
            
            %Position
            if(j<=length(polynomial_r) && (k<=length(polynomial_r)))
                order_t_r = ((order-k_r-j+1)+(order-k_r-k+1));
                if(j==k)
                    A_y(j,k) = polynomial_r(j)^2/(order_t_r+1) * (t(i+1)^(order_t_r+1)-t(i)^(order_t_r+1));
                    A_z(j,k) = polynomial_r(j)^2/(order_t_r+1) * (t(i+1)^(order_t_r+1)-t(i)^(order_t_r+1));
                else
                    A_y(j,k) = 2*polynomial_r(j)*polynomial_r(k)/(order_t_r+1)*(t(i+1)^(order_t_r+1)-t(i)^(order_t_r+1));
                    A_z(j,k) = 2*polynomial_r(j)*polynomial_r(k)/(order_t_r+1)*(t(i+1)^(order_t_r+1)-t(i)^(order_t_r+1));
                end
            end

        end
    end
    
    H = blkdiag(H,mu_r*A_y,mu_r*A_z);
end

H = 0.5*(H + H.'); %Make it symmetric
end