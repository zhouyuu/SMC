function outdata = ZeroPfilter(Wp,Ws,Ap,As,x)
% Zero-Phase Filter base on buttor(基于巴特沃斯模拟滤波器的零相位滤波)
%-------------------------------------
% outdata = ZeroPfilter(Wp,Ws,Ap,As,x)
% outdata: the filted signal
% [Wp,Ws,Ap,As]: parameter of buttor filter
% x: the origin signal
% by noWen(NNU_GIS)
% EXAMPLE:
% x = randn(480,1);
% out = ZeroPfilter(0.05*pi,0.1*pi,1,15,x);
% plot(x); hold all ;plot(out);
% legend('oridata','filterdata');
%-------------------------------------

%－－－－－有限冲激响应滤波器的边界处理(此处使用巴特沃斯模拟滤波器，不用正理边界)
% N = length(x);
% M = 30; %n of the filter
% xx = ones(N+M,1);
% for i = 1 :N
%     xx(i)=x(i);
% end
% for i = 1 : M
%     xx(N+i)= 2*xx(N)-xx(N-i);
% end
%  x = xx;
%--------------------------------------------------------------------------

%-------------design of buttor filter(巴特沃斯模拟滤波器设计)---------------
%判断滤波方式
if length(Wp)==2 
    s = 'bandpass' ;   %带通
    else if Wp > Ws
        s = 'high' ;   %高通
    else if Wp < Ws
        s = 'low'  ;   %低通   
    end
    end
end
Ap = Ap/2; As = As/2;  %衰减速率减半（因为下面有两次滤波处理）
[N,Wn]=buttord(Wp/pi,Ws/pi,Ap,As); %计算巴特沃斯滤波器阶次和截止频率
[b,a]=butter(N,Wn,s);              %频率变换法设计巴特沃斯带通滤波器
%--------------------------------------------------------------------------

%---------------零相位滤波实现----------------
y = filter(b,a,x);  %第一次滤波 
y = fliplr(y')';    %滤波结果反转
y = filter(b,a,y);  %第二次滤波
y = fliplr(y')';    %再次反转，即为最终结果

%------------边界处理相关部分--------------
% for i = 1 :N
%     yy(i)=y(i);
% end
% y = yy;
%----------------------------------------

outdata = y';  %结果输出

end

