close all;

m=1;

x_poly = solution(1+0*(order+1):(order+1)+0*(order+1));
y_poly = solution(1+1*(order+1):(order+1)+1*(order+1));

x_trajec   = [polyval(x_poly, t(1):0.01:t(2)) 2*acrob_center(2)-polyval(x_poly, flip(t(1):0.01:t(2)))];
y_trajec   = [polyval(y_poly, t(1):0.01:t(2)) polyval(y_poly, flip(t(1):0.01:t(2)))];
time       = [t(1):0.01:t(2) t(2):0.01:t(2)*2];

%%
figure(1);
plot(x_trajec,y_trajec,'-k');

%%
figure(2);
subplot(2,1,1); plot(time,x_trajec,'-k'); ylabel('x'); hold on;
subplot(2,1,2); plot(time,y_trajec,'-k'); ylabel('y'); hold on;


%%
x_trajec_d   = [polyval(polyder(x_poly), t(1):0.01:t(2))  polyval(polyder(x_poly), flip(t(1):0.01:t(2)))];
y_trajec_d   = [polyval(polyder(y_poly), t(1):0.01:t(2)) -polyval(polyder(y_poly), flip(t(1):0.01:t(2)))];
 
figure(3);
subplot(2,1,1); plot(time,x_trajec_d,'-k'); ylabel('v_x'); hold on;
subplot(2,1,2); plot(time,y_trajec_d,'-k'); ylabel('v_y'); hold on;

%%
x_trajec_d2   = [polyval(polyder(polyder(x_poly)),t(1):0.01:t(2)) -polyval(polyder(polyder(x_poly)),flip(t(1):0.01:t(2)))];
y_trajec_d2   = [polyval(polyder(polyder(y_poly)),t(1):0.01:t(2))  polyval(polyder(polyder(y_poly)),flip(t(1):0.01:t(2)))];

figure(4);
subplot(2,1,1); plot(time,x_trajec_d2,'-k'); ylabel('a_x'); hold on;
subplot(2,1,2); plot(time,y_trajec_d2,'-k'); ylabel('a_y'); hold on;

%%
x_trajec_d3   = [polyval(polyder(polyder(polyder(x_poly))),t(1):0.01:t(2))  polyval(polyder(polyder(polyder(x_poly))),flip(t(1):0.01:t(2)))];
y_trajec_d3   = [polyval(polyder(polyder(polyder(y_poly))),t(1):0.01:t(2)) -polyval(polyder(polyder(polyder(y_poly))),flip(t(1):0.01:t(2)))];

figure(5);
subplot(2,1,1); plot(time,x_trajec_d3,'-k'); ylabel('j_x'); hold on;
subplot(2,1,2); plot(time,y_trajec_d3,'-k'); ylabel('j_y'); hold on;
