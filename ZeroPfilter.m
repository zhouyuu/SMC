function outdata = ZeroPfilter(Wp,Ws,Ap,As,x)
% Zero-Phase Filter base on buttor(���ڰ�����˹ģ���˲���������λ�˲�)
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

%�������������޳弤��Ӧ�˲����ı߽紦��(�˴�ʹ�ð�����˹ģ���˲�������������߽�)
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

%-------------design of buttor filter(������˹ģ���˲������)---------------
%�ж��˲���ʽ
if length(Wp)==2 
    s = 'bandpass' ;   %��ͨ
    else if Wp > Ws
        s = 'high' ;   %��ͨ
    else if Wp < Ws
        s = 'low'  ;   %��ͨ   
    end
    end
end
Ap = Ap/2; As = As/2;  %˥�����ʼ��루��Ϊ�����������˲�����
[N,Wn]=buttord(Wp/pi,Ws/pi,Ap,As); %���������˹�˲����״κͽ�ֹƵ��
[b,a]=butter(N,Wn,s);              %Ƶ�ʱ任����ư�����˹��ͨ�˲���
%--------------------------------------------------------------------------

%---------------����λ�˲�ʵ��----------------
y = filter(b,a,x);  %��һ���˲� 
y = fliplr(y')';    %�˲������ת
y = filter(b,a,y);  %�ڶ����˲�
y = fliplr(y')';    %�ٴη�ת����Ϊ���ս��

%------------�߽紦����ز���--------------
% for i = 1 :N
%     yy(i)=y(i);
% end
% y = yy;
%----------------------------------------

outdata = y';  %������

end

