function CartPole4DOF_GUI
% CARTPOLE4DOF_GUI - 4-DOF spatial cart-pole demonstration (base MATLAB).
% A single rigid rod tilts toward X and Y. Cart translates in the XY plane.
% Two forces Fx and Fy allow local control of both tilt directions.
% Same isometric perspective camera in both animation columns.
    mode=4; %#ok<NASGU>
    initLab();

    function initLab()
    C.bg=[.949 .962 .980]; C.ink=[.095 .15 .24];
    C.muted=[.37 .45 .55]; C.white=[1 1 1];
    C.pd=[.18 .40 .77]; C.lqr=[.04 .57 .51];
    C.burg=[.51 .075 .23]; C.purple=[.65 .32 .67];
    C.line=[.82 .87 .93]; C.amber=[.81 .46 .12];
    ctlColor=[C.pd;C.lqr];
    H=struct(); F=struct(); D=[]; token=0; paused=false; running=false;
    ss=get(0,'ScreenSize'); fw=min(1750,max(1190,ss(3)-75));
    fh=min(1060,max(825,ss(4)-100));
    fig=figure('Name','CART-POLE LAB | 4-DOF SPATIAL | PD versus LQR', ...
       'Color',C.bg,'NumberTitle','off','MenuBar','none','ToolBar','none', ...
       'Units','pixels','Position',[max(25,(ss(3)-fw)/2),max(40,(ss(4)-fh)/2),fw,fh], ...
       'CloseRequestFcn',@closeUI,'Resize','on');
    uiText(fig,[.013 .952 .62 .037],'SPATIAL CART-POLE  /  4-DOF LAB',17,C.ink,'bold');
    uiText(fig,[.014 .929 .78 .020], ...
       'ONE RIGID ROD  |  TWO PLANAR ACTUATORS  |  TWO SPATIAL TILTS  |  3D PERSPECTIVE',8.5,C.muted,'normal');
    uiText(fig,[.81 .957 .17 .02],'PD  vs  LQR',9,C.muted,'bold','right');

    P=panel([.012 .731 .210 .187],'01 / PLANT PROPERTIES',C.pd);
    F.M=smallField(P,0,.69,'Cart M','1'); F.m=smallField(P,1,.69,'Rod m','0.2');
    F.L=smallField(P,0,.47,'Length L','0.5'); F.g=smallField(P,1,.47,'g','9.81');
    F.frac=smallField(P,0,.25,'COM/L','0.5'); F.ig=smallField(P,1,.25,'IG/(mL2)','0.0833333');
    uiText(P,[.05 .04 .9 .10],'Rod inertial properties can be modified independently.',7.4,C.muted,'normal');

    I=panel([.229 .731 .210 .187],'02 / INITIAL CONDITIONS',C.pd);
    if mode==3
        F.x0=smallField(I,0,.71,'x (m)','0'); F.vx0=smallField(I,1,.71,'vx (m/s)','0');
        F.a0=smallField(I,0,.49,'Tilt X (deg)','5'); F.b0=smallField(I,1,.49,'Tilt Y (deg)','3');
        F.wa0=smallField(I,0,.27,'Rate X','0'); F.wb0=smallField(I,1,.27,'Rate Y','0');
        uiText(I,[.05 .05 .9 .11],'Rates in degrees/s. Y tilt is uncontrolled.',7.4,C.burg,'normal');
    else
        F.x0=smallField(I,0,.75,'x (m)','0');F.y0=smallField(I,1,.75,'y (m)','0');
        F.a0=smallField(I,0,.555,'Tilt X','5');F.b0=smallField(I,1,.555,'Tilt Y','3');
        F.vx0=smallField(I,0,.36,'vx (m/s)','0');F.vy0=smallField(I,1,.36,'vy (m/s)','0');
        F.wa0=smallField(I,0,.165,'Rate X','0');F.wb0=smallField(I,1,.165,'Rate Y','0');
    end

    PP=panel([.446 .731 .196 .187],'03 / PD POLE TARGETS',C.pd);
    uiText(PP,[.10 .80 .20 .13],'Pole',8,C.muted,'bold');
    uiText(PP,[.38 .80 .25 .13],'Real',8,C.muted,'bold');
    uiText(PP,[.72 .80 .27 .13],'Imag.',8,C.muted,'bold');
    F.pr=cell(4,1);F.pi=cell(4,1);
    dre={'-2','-3','-0.5','-0.5'};dim={'0','0','4','-4'};
    for k=1:4
       y=.64-(k-1)*.177;
       uiText(PP,[.09 y .18 .14],sprintf('s%d',k),8.5,C.ink,'bold');
       F.pr{k}=edit(PP,[.30 y-.005 .30 .18],dre{k});
       F.pi{k}=edit(PP,[.64 y-.005 .30 .18],dim{k});
    end

    LQR=panel([.649 .731 .177 .187],'04 / LQR WEIGHTS',C.lqr);
    F.qpos=smallField(LQR,0,.72,'Q: cart','10');
    F.qvel=smallField(LQR,1,.72,'Q: speed','1');
    F.qtilt=smallField(LQR,0,.49,'Q: tilt','100');
    F.qrate=smallField(LQR,1,.49,'Q: rate','1');
    F.r=smallField(LQR,0,.25,'R: Fx','1');
    if mode==3
        uiText(LQR,[.53 .21 .45 .18],'Only controllable X states enter LQR',7,C.muted,'normal','center');
    else
        F.ry=smallField(LQR,1,.25,'R: Fy','1');
    end

    S=panel([.833 .731 .155 .187],'05 / RUN + FORCE',C.pd);
    F.T=smallField(S,0,.72,'Time (s)','6');
    F.speed=smallField(S,1,.72,'Speed','1');
    if mode==3
        F.fxMax=smallField(S,0,.50,'Max Fx','25');
        F.trail=smallField(S,1,.50,'Dots s','0.5');
        F.pulseX=smallField(S,0,.28,'Pulse X','0');
        F.pulseAt=smallField(S,1,.28,'At (s)','0.8');
        F.pulseY=[];
        uiText(S,[.05 .05 .90 .12],'Pulse width 0.15 s. Single actuator.',7.3,C.muted,'normal');
    else
        F.fxMax=smallField(S,0,.51,'Max Fx','25');
        F.fyMax=smallField(S,1,.51,'Max Fy','25');
        F.pulseX=smallField(S,0,.31,'Pulse X','0');
        F.pulseY=smallField(S,1,.31,'Pulse Y','0');
        F.pulseAt=smallField(S,0,.11,'At (s)','0.8');
        F.trail=smallField(S,1,.11,'Dots s','0.5');
    end

    H.go=push(fig,[.014 .674 .144 .041],'SIMULATE BOTH',C.ink,C.white,@onRun);
    H.pause=push(fig,[.168 .674 .083 .041],'PAUSE',[.81 .87 .94],C.ink,@onPause);
    H.reset=push(fig,[.260 .674 .080 .041],'RESET',[.81 .87 .94],C.ink,@onReset);
    H.tune=push(fig,[.350 .674 .116 .041],'AUTO-DESIGN PD',C.pd,C.white,@onDesign);
    H.export=push(fig,[.478 .674 .092 .041],'EXPORT',C.lqr,C.white,@onExport);
    if mode==3
        statusText='Rank 4/6: transverse tilt cannot be stabilized by Fx.';
    else
        statusText='Rank 8/8: two-axis linearized system is controllable.';
    end
    H.status=uiText(fig,[.589 .677 .397 .036],statusText,8.5,C.muted,'bold');

    left=[.012 .507]; columnW=.481;
    for j=1:2
        xx=left(j); cc=ctlColor(j,:);
        top=uipanel('Parent',fig,'Units','normalized','Position',[xx .626 columnW .040], ...
            'BorderType','none','BackgroundColor',cc);
        if j==1, hname='PD / POLE PLACEMENT';else,hname='LQR / OPTIMAL FEEDBACK';end
        uiText(top,[.013 .12 .63 .75],hname,10,C.white,'bold');
        H.read(j)=uiText(top,[.58 .11 .40 .76],'INITIAL',8,C.white,'normal','right');
        ac=axesPanel([xx .318 columnW .300]);
        H.anim(j)=axes('Parent',ac,'Units','normalized','Position',[.090 .10 .87 .85], ...
            'Color','w','FontSize',8.5,'Box','off','XColor',C.ink,'YColor',C.ink,'ZColor',C.ink);
        xlabel(H.anim(j),'X, cart travel (m)');ylabel(H.anim(j),'Y, transverse (m)');
        zlabel(H.anim(j),'Z, elevation (m)');
        chartY=[.158 .010]; chartTitle={'ROD TILT ANGLES','CONTROL FORCE'};
        for q=1:2
            ac=axesPanel([xx chartY(q) columnW .142]);
            uiText(ac,[.023 .873 .68 .11],chartTitle{q},8.5,cc,'bold');
            a=axes('Parent',ac,'Units','normalized','Position',[.09 .27 .868 .65], ...
               'Color','w','FontSize',8,'Box','off','XColor',C.ink,'YColor',C.ink);
            grid(a,'on');hold(a,'on');set(a,'GridAlpha',.12);
            xlabel(a,'Time (s)');
            if q==1,ylabel(a,'Tilt (deg)');else
               if mode==3,ylabel(a,'F_x (N)');else,ylabel(a,'Forces (N)');end
            end
            H.graph(j,q)=a;
        end
    end
    onReset([],[]);

    function p=param()
        p.nc=mode-2;p.nq=p.nc+2;
        p.M=num(F.M,'M');p.m=num(F.m,'m');p.L=num(F.L,'L');p.g=num(F.g,'g');
        p.frac=num(F.frac,'COM/L');p.ig=num(F.ig,'IG coefficient');
        p.lc=p.frac*p.L;p.IG=p.ig*p.m*p.L^2;p.J=p.IG+p.m*p.lc^2;
        p.a=p.M+p.m;p.b=p.m*p.lc;p.k=p.m*p.g*p.lc;
        p.T=num(F.T,'duration');p.speed=num(F.speed,'speed');
        p.trail=num(F.trail,'trail');
        p.dx=num(F.pulseX,'pulse X');p.td=num(F.pulseAt,'pulse time');
        p.x0=num(F.x0,'x0');p.vx0=num(F.vx0,'vx0');
        if mode==3
            p.fmax=num(F.fxMax,'force limit');p.dy=0;
            p.y0=0;p.vy0=0;
        else
            p.fmax=[num(F.fxMax,'Fx limit');num(F.fyMax,'Fy limit')];
            p.dy=num(F.pulseY,'pulse Y');
            p.y0=num(F.y0,'y0');p.vy0=num(F.vy0,'vy0');
        end
        p.al0=num(F.a0,'tilt X')*pi/180;p.be0=num(F.b0,'tilt Y')*pi/180;
        p.wa0=num(F.wa0,'rate X')*pi/180;p.wb0=num(F.wb0,'rate Y')*pi/180;
        if mode==3
            p.z0=[p.x0;p.al0;p.be0;p.vx0;p.wa0;p.wb0];
        else
            p.z0=[p.x0;p.y0;p.al0;p.be0;p.vx0;p.vy0;p.wa0;p.wb0];
        end
        p.q=[num(F.qpos,'Q cart'),num(F.qvel,'Q speed'),num(F.qtilt,'Q tilt'),num(F.qrate,'Q rate')];
        if mode==3,p.R=num(F.r,'R force');
        else,p.R=[num(F.r,'R Fx');num(F.ry,'R Fy')];end
        p.poles=zeros(4,1);
        for kk=1:4, p.poles(kk)=num(F.pr{kk},'pole real')+1i*num(F.pi{kk},'pole imaginary');end
        if any([p.M,p.m,p.L,p.g,p.IG,p.J,p.T,p.speed,p.fmax(:)',p.R(:)']<=0) || ...
           p.frac<=0 || p.frac>=1 || p.trail<0 || any(p.q<0) || ...
           p.T>25 || p.speed>20
           error('Masses, length, gravity, inertia, duration, speed, force limit and R must be positive. COM/L must be between 0 and 1.');
        end
        if max(abs([p.al0 p.be0]))>1.25
           error('For upright linear feedback, start within about 70 degrees of upright. This is not a swing-up GUI.');
        end
    end
    function val=num(h,label)
        val=str2double(get(h,'String'));
        if ~isscalar(val)||~isfinite(val),error('Invalid numeric field: %s',label);end
    end
    function [A2,B2]=planar(p)
        d=p.a*p.J-p.b^2;
        if d<=0,error('Nonpositive inertia determinant.');end
        A2=[0 1 0 0;0 0 -p.b*p.k/d 0;0 0 0 1;0 0 p.a*p.k/d 0];
        B2=[0;p.J/d;0;-p.b/d];
    end
    function k=placePD(p)
        planar(p);
        roots=p.poles;
        if any(real(roots)>=0),error('All PD poles must have negative real parts.');end
        if max(abs(sort(roots)-sort(conj(roots))))>1e-7
           % sort of complex numbers need not align conjugates in all versions;
           % use nearest-conjugate matching instead.
           for ii=1:4
             if min(abs(roots-conj(roots(ii))))>1e-6
                error('Complex desired poles must form conjugate pairs.');
             end
           end
        end
        v=poly(roots);v=real(v);d=p.a*p.J-p.b^2;
        kx=-v(5)*d/(p.g*p.b); kv=-v(4)*d/(p.g*p.b);
        kth=(p.J*kx-p.a*p.k-v(3)*d)/p.b;
        kw=(p.J*kv-v(2)*d)/p.b;
        k=[kx kv kth kw];
        [A2,B2]=planar(p);z=eig(A2-B2*k);
        vAct=real(poly(z));
        if norm(vAct-v,inf)>max(1, norm(v,inf))*1e-5
            error('Pole placement is poorly conditioned for these parameters.');
        end
    end
    function k=designLQR(p)
        if mode==3
            [AA,BB]=planar(p);Q=diag(p.q);R=p.R;
        else
            [MM,~]=model(p,zeros(2*p.nq,1));
            grav=zeros(p.nq);grav(p.nc+1,p.nc+1)=-p.k;
            grav(p.nc+2,p.nc+2)=-p.k;
            AA=[zeros(p.nq) eye(p.nq);-MM\grav zeros(p.nq)];
            BB=[zeros(p.nq,p.nc); MM\[eye(p.nc);zeros(2,p.nc)]];
            Q=diag([p.q(1)*ones(1,2),p.q(3)*ones(1,2), ...
                    p.q(2)*ones(1,2),p.q(4)*ones(1,2)]);
            R=diag(p.R);
        end
        nn=size(AA,1);
        hh=[AA -BB*(R\BB');-Q -AA'];
        [V,ev]=eig(hh);[~,idx]=sort(real(diag(ev)));
        if max(real(diag(ev(idx(1:nn),idx(1:nn)))))>=-1e-7
            error('LQR could not identify a stabilizing invariant subspace.');
        end
        U=V(:,idx(1:nn));X=U(1:nn,:);Y=U(nn+1:end,:);
        if rcond(X)<1e-12,error('LQR Riccati eigenvectors are ill-conditioned.');end
        P=real(Y/X);P=(P+P')/2;k=(R\(BB'*P));
        if max(real(eig(AA-BB*k)))>=-1e-7,error('LQR result not stabilizing.');end
    end
    function kMat=fullK(p,k)
        if mode==3
            % z=[x;alpha;beta;vx;wa;wb]. No actuation of beta.
            kMat=zeros(1,6);kMat([1 4 2 5])=k;
        else
            % z=[x;y;alpha;beta;vx;vy;wa;wb]
            kMat=zeros(2,8);
            kMat(1,[1 5 3 7])=k;
            kMat(2,[2 6 4 8])=k;
        end
    end
    function [MM,h0]=model(p,z)
        nc=p.nc;nq=p.nq;al=z(nc+1);be=z(nc+2);
        da=z(nq+nc+1);db=z(nq+nc+2);
        ca=cos(al);sa=sin(al);cb=cos(be);sb=sin(be);
        nxA=ca*cb;nxB=-sa*sb;
        MM=zeros(nq);MM(1,1)=p.a;
        MM(1,nc+1)=p.b*nxA;MM(nc+1,1)=p.b*nxA;
        MM(1,nc+2)=p.b*nxB;MM(nc+2,1)=p.b*nxB;
        if mode==4
            MM(2,2)=p.a;
            MM(2,nc+2)=p.b*cb;MM(nc+2,2)=p.b*cb;
        end
        MM(nc+1,nc+1)=p.J*cb^2;MM(nc+2,nc+2)=p.J;
        h0=zeros(nq,1);
        h0(1)=-p.b*(sa*cb*(da^2+db^2)+2*ca*sb*da*db);
        if mode==4,h0(2)=-p.b*sb*db^2;end
        h0(nc+1)=-2*p.J*sb*cb*da*db-p.k*sa*cb;
        h0(nc+2)=p.J*sb*cb*da^2-p.k*ca*sb;
    end
    function val=rhs(t,z,p,kmat)
        [MM,h0]=model(p,z);
        uu=control(z,p,kmat);dist=zeros(p.nc,1);
        if t>=p.td && t<p.td+.15
            dist(1)=p.dx;
            if mode==4,dist(2)=p.dy;end
        end
        ff=[uu(:)+dist;zeros(2,1)];acc=MM\(ff-h0);
        val=[z(p.nq+1:end);acc];
    end
    function uu=control(z,p,kmat)
        raw=-kmat*z(:);
        uu=max(-p.fmax,min(p.fmax,raw));
    end
    function [value,stop,direction]=endEvent(~,z,p)
        value=1.28-max(abs(z(p.nc+1:p.nc+2)));
        stop=1;direction=-1;
    end
    function out=simulate(p,kmat)
        tt=linspace(0,p.T,max(260,ceil(p.T/.016)));
        oo=odeset('RelTol',1e-7,'AbsTol',1e-9,'Events',@(t,z)endEvent(t,z,p));
        [time,zz]=ode45(@(t,z)rhs(t,z,p,kmat),tt,p.z0,oo);
        ff=zeros(numel(time),p.nc);
        for ii=1:numel(time),ff(ii,:)=control(zz(ii,:)',p,kmat)';end
        out=struct('t',time,'z',zz,'u',ff,'hit',time(end)<p.T-.02);
    end
    function onDesign(~,~)
        try
            p=param();k=placePD(p);
            if mode==3,ranktext='Rank 4/6: Y tilt unstable';
            else,ranktext='Rank 8/8: X/Y controlled';end
            set(H.status,'String',sprintf( ...
              'PD gains [%.3g %.3g %.3g %.3g] | %s',k,ranktext));
        catch ME,errordlg(ME.message,'Pole design');end
    end
    function onRun(~,~)
        token=token+1;this=token;paused=false;running=true;
        set(H.pause,'String','PAUSE');
        try
            p=param();kp=fullK(p,placePD(p));
            if mode==3,kl=fullK(p,designLQR(p));
            else,kl=designLQR(p);end
            D=struct();D.p=p;D.Kp=kp;D.Kl=kl;
            D.pd=simulate(p,kp);D.lqr=simulate(p,kl);
            if mode==3
                set(H.status,'String', ...
                   'Rank 4/6. X-tilt recoverable; transverse Y tilt is open-loop unstable.');
            else
                set(H.status,'String', ...
                   'Rank 8/8. Both tilts locally stabilizable with Fx and Fy.');
            end
            setupPlots();animate(this);
        catch ME
            running=false;set(H.status,'String',['Error: ' ME.message]);
            errordlg(ME.message,'Simulation');
        end
        running=false;
    end
    function setupPlots()
        p=D.p;O={D.pd,D.lqr};
        allX=[D.pd.z(:,1);D.lqr.z(:,1);0];
        limx=max(1.45*p.L,max(abs(allX))+1.1*p.L);
        if mode==3,limy=1.5*p.L;
        else
            allY=[D.pd.z(:,2);D.lqr.z(:,2);0];
            limy=max(1.45*p.L,max(abs(allY))+1.1*p.L);
        end
        limz=[-.08*p.L 2.15*p.L];
        tmax=max([D.pd.t(end),D.lqr.t(end),.5]);
        angl=zeros(0,1);force=zeros(0,1);
        for jj=1:2
            angl=[angl;O{jj}.z(:,p.nc+1)*180/pi;O{jj}.z(:,p.nc+2)*180/pi]; %#ok<AGROW>
            force=[force;O{jj}.u(:)]; %#ok<AGROW>
        end
        alim=max(8,max(abs(angl))*1.14);flim=max(2,max(abs(force))*1.20);
        for jj=1:2
            a=H.anim(jj);cla(a);hold(a,'on');grid(a,'on');
            xlim(a,[-limx limx]);ylim(a,[-limy limy]);zlim(a,limz);
            axis(a,'manual');daspect(a,[1 1 1]);
            view(a,42,24);camproj(a,'perspective');
            set(a,'DataAspectRatioMode','manual','GridAlpha',.12);
            xlabel(a,'X (m)');ylabel(a,'Y (m)');zlabel(a,'Z (m)');
            drawGround(a,limx,limy,p);
            ac=H.graph(jj,1);fc=H.graph(jj,2);
            cla(ac);hold(ac,'on');grid(ac,'on');
            cla(fc);hold(fc,'on');grid(fc,'on');
            xlim(ac,[0 tmax]);ylim(ac,[-alim alim]);
            xlim(fc,[0 tmax]);ylim(fc,[-flim flim]);
            H.al(jj)=plot(ac,nan,nan,'-','Color',C.pd,'LineWidth',1.8);
            H.be(jj)=plot(ac,nan,nan,'-','Color',C.purple,'LineWidth',1.8);
            H.forceline(jj)=plot(fc,nan,nan,'-','Color',C.pd,'LineWidth',1.8);
            if mode==4
                H.forceY(jj)=plot(fc,nan,nan,'-','Color',C.purple,'LineWidth',1.8);
            end
            H.ta(jj)=plot(ac,[0 0],[-alim alim],':','Color',C.muted);
            H.tf(jj)=plot(fc,[0 0],[-flim flim],':','Color',C.muted);
            legend(ac,{'Tilt toward X','Tilt toward Y'},'Location','northwest','FontSize',7.5);
            if mode==4
               legend(fc,{'Force X','Force Y'},'Location','northwest','FontSize',7.5);
            end
            if jj==1,kk=kpTxt();else,kk=klTxt();end
            makeObjects(jj,O{jj}.z(1,:)',p,kk);
        end
    end
    function a=kpTxt(),a='PD';end
    function a=klTxt(),a='LQR';end
    function drawGround(ax,rx,ry,p)
        [xx,yy]=meshgrid(linspace(-rx,rx,5),linspace(-ry,ry,5));
        surf(ax,xx,yy,-.045*p.L+zeros(size(xx)), ...
            'FaceColor',[.94 .96 .98],'FaceAlpha',.25,'EdgeColor',C.line);
        plot3(ax,[-rx rx],[0 0],[0 0],'Color',[.58 .65 .76],'LineWidth',2.1);
        if mode==3
            plot3(ax,[-rx rx],[.17*p.L .17*p.L],[0 0], ...
                'Color',[.72 .78 .84],'LineWidth',1.5);
            plot3(ax,[-rx rx],[-.17*p.L -.17*p.L],[0 0], ...
                'Color',[.72 .78 .84],'LineWidth',1.5);
        else
            plot3(ax,[0 0],[-ry ry],[0 0],'Color',[.58 .65 .76],'LineWidth',2.1);
        end
    end
    function makeObjects(jj,z,p,controllerName)
        ax=H.anim(jj); cc=ctlColor(jj,:);
        x=z(1);
        if mode==3,y=0;else,y=z(2);end
        q=z(p.nc+1:p.nc+2);point=tip(x,y,q,p);
        w=.40*p.L;dep=.32*p.L;thick=.105*p.L;
        V=boxVerts([x,y,.055*p.L],[w,dep,thick]);
        o.cart=patch(ax,'Vertices',V,'Faces', ...
            [1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8], ...
            'FaceColor',[.33 .43 .56],'EdgeColor',[.18 .27 .39], ...
            'FaceAlpha',1,'LineWidth',.65);
        [sx,sy,sz]=sphere(9);r=.031*p.L;
        o.wheel=gobjects(4,1);
        for widx=1:4
            sgnx=2*mod(widx,2)-1;sgny=2*(widx>2)-1;
            ox=x+sgnx*.155*p.L;oy=y+sgny*.135*p.L;
            o.wheel(widx)=surf(ax,ox+r*sx,oy+r*sy,r*sz+.033*p.L, ...
                'FaceColor',[.18 .24 .33],'EdgeColor','none');
        end
        py=.055*p.L+thick;
        o.rod=plot3(ax,[x point(1)],[y point(2)],[py point(3)+py],'-', ...
                 'LineWidth',5,'Color',cc);
        o.pivot=plot3(ax,x,y,py,'o','MarkerSize',7,'MarkerFaceColor','w', ...
                     'MarkerEdgeColor',C.ink,'LineWidth',1.5);
        o.tip=plot3(ax,point(1),point(2),point(3)+py,'o', ...
                  'MarkerSize',9,'MarkerFaceColor','w','MarkerEdgeColor',cc, ...
                  'LineWidth',1.8);
        o.dots=plot3(ax,nan,nan,nan,'o','LineStyle','none','MarkerSize',3.5, ...
                  'MarkerEdgeColor',C.burg,'MarkerFaceColor',C.burg);
        o.arrow=quiver3(ax,x,y,py,0,0,0,0,'Color',C.amber,'LineWidth',1.9, ...
                      'MaxHeadSize',.8);
        if mode==4
            o.arrowY=quiver3(ax,x,y,py,0,0,0,0,'Color',C.purple, ...
                            'LineWidth',1.9,'MaxHeadSize',.8);
        end
        o.geo=[w dep thick py];o.p=p;o.name=controllerName;
        H.obj(jj)=o;
    end
    function point=tip(x,y,rot,p)
        al=rot(1);be=rot(2);
        point=[x+p.L*sin(al)*cos(be),y+p.L*sin(be),p.L*cos(al)*cos(be)];
    end
    function V=boxVerts(center,dim)
        x=center(1);y=center(2);z=center(3);
        a=dim(1)/2;b=dim(2)/2;c=dim(3);
        V=[x-a y-b z;x+a y-b z;x+a y+b z;x-a y+b z; ...
           x-a y-b z+c;x+a y-b z+c;x+a y+b z+c;x-a y+b z+c];
    end
    function animate(this)
        if isempty(D),return;end
        p=D.p;O={D.pd,D.lqr};
        dt=.018;tcommon=0:dt:max(D.pd.t(end),D.lqr.t(end));
        for n=1:numel(tcommon)
            if ~ishandle(fig)||this~=token,return;end
            while paused && this==token && ishandle(fig),pause(.06);drawnow;end
            for jj=1:2
               idx=find(O{jj}.t<=tcommon(n),1,'last');
               if isempty(idx),idx=1;end
               setFrame(jj,idx,O{jj},p);
            end
            drawnow limitrate;
            if n<numel(tcommon),pause(max(.002,dt/max(.2,p.speed)));end
        end
        if ishandle(fig)
           if D.pd.hit||D.lqr.hit
              if mode==3,txt='Tilt limit reached. Transverse mode cannot be stabilized by Fx.';
              else,txt='Tilt limit reached. Try smaller initial angles or larger force limits.';end
           else
              if mode==3,txt='Simulation complete. Observe the uncontrolled transverse tilt.';
              else,txt='Simulation complete. Both spatial tilt directions regulated.';end
           end
           set(H.status,'String',txt);
        end
    end
    function setFrame(jj,idx,out,p)
        z=out.z(idx,:)';x=z(1);
        if mode==3,y=0;else,y=z(2);end
        al=z(p.nc+1);be=z(p.nc+2);
        rot=[al be];o=H.obj(jj);py=o.geo(4);point=tip(x,y,rot,p);
        set(o.cart,'Vertices',boxVerts([x y .055*p.L],o.geo(1:3)));
        r=.031*p.L;[sx,sy,sz]=sphere(9);
        for widx=1:4
          sgnx=2*mod(widx,2)-1;sgny=2*(widx>2)-1;
          ox=x+sgnx*.155*p.L;oy=y+sgny*.135*p.L;
          set(o.wheel(widx),'XData',ox+r*sx,'YData',oy+r*sy,'ZData',r*sz+.033*p.L);
        end
        set(o.rod,'XData',[x point(1)],'YData',[y point(2)],'ZData',[py point(3)+py]);
        set(o.pivot,'XData',x,'YData',y,'ZData',py);
        set(o.tip,'XData',point(1),'YData',point(2),'ZData',point(3)+py);
        first=find(out.t>=out.t(idx)-p.trail,1);
        ids=first:4:idx;if isempty(ids),ids=idx;end
        xTra=out.z(ids,1)+p.L*sin(out.z(ids,p.nc+1)).*cos(out.z(ids,p.nc+2));
        if mode==3,yTra=p.L*sin(out.z(ids,p.nc+2));
        else,yTra=out.z(ids,2)+p.L*sin(out.z(ids,p.nc+2));end
        zTra=py+p.L*cos(out.z(ids,p.nc+1)).*cos(out.z(ids,p.nc+2));
        set(o.dots,'XData',xTra,'YData',yTra,'ZData',zTra);
        set(o.arrow,'XData',x,'YData',y,'ZData',py, ...
            'UData',sign(out.u(idx,1))*.27*p.L*min(abs(out.u(idx,1))/p.fmax(1),1), ...
            'VData',0,'WData',0);
        if mode==4
            set(o.arrowY,'XData',x,'YData',y,'ZData',py,'UData',0, ...
                'VData',sign(out.u(idx,2))*.27*p.L*min(abs(out.u(idx,2))/p.fmax(2),1), ...
                'WData',0);
        end
        set(H.al(jj),'XData',out.t(1:idx),'YData',out.z(1:idx,p.nc+1)*180/pi);
        set(H.be(jj),'XData',out.t(1:idx),'YData',out.z(1:idx,p.nc+2)*180/pi);
        set(H.forceline(jj),'XData',out.t(1:idx),'YData',out.u(1:idx,1));
        if mode==4
            set(H.forceY(jj),'XData',out.t(1:idx),'YData',out.u(1:idx,2));
        end
        set(H.ta(jj),'XData',[out.t(idx) out.t(idx)]);
        set(H.tf(jj),'XData',[out.t(idx) out.t(idx)]);
        set(H.read(jj),'String',sprintf('t %.2fs | X %+4.1f%c | Y %+4.1f%c', ...
              out.t(idx),al*180/pi,char(176),be*180/pi,char(176)));
    end
    function onPause(~,~)
        paused=~paused;if paused,set(H.pause,'String','RESUME'); ...
            else,set(H.pause,'String','PAUSE');end
    end
    function onReset(~,~)
        token=token+1;running=false;paused=false;D=[];
        set(H.pause,'String','PAUSE');
        try
          p=param();O0=p.z0;
          for jj=1:2
            a=H.anim(jj);cla(a);hold(a,'on');grid(a,'on');
            xlim(a,[-max(1,2*p.L) max(1,2*p.L)]);
            ylim(a,[-max(1,2*p.L) max(1,2*p.L)]);zlim(a,[-.08*p.L 2.15*p.L]);
            daspect(a,[1 1 1]);view(a,42,24);camproj(a,'perspective');
            xlabel(a,'X (m)');ylabel(a,'Y (m)');zlabel(a,'Z (m)');
            drawGround(a,max(1,2*p.L),max(1,2*p.L),p);
            makeObjects(jj,O0,p,'INITIAL');
            for q=1:2
              g=H.graph(jj,q);cla(g);grid(g,'on');
              xlim(g,[0 max(p.T,1)]);ylim(g,[-10 10]);
              xlabel(g,'Time (s)');
              if q==1,ylabel(g,'Tilt (deg)');
              elseif mode==3,ylabel(g,'F_x (N)');
              else,ylabel(g,'Forces (N)');end
            end
            set(H.read(jj),'String','INITIAL');
          end
        catch ME
          set(H.status,'String',ME.message);
        end
    end
    function onExport(~,~)
        if isempty(D),errordlg('Run a simulation before exporting.');return;end
        [fn,fp]=uiputfile(sprintf('CartPole%dDOF_results.mat',mode),'Save numerical results');
        if isequal(fn,0),return;end
        result=D;save(fullfile(fp,fn),'result');
    end
    function closeUI(~,~),token=token+1;delete(fig);end
    function pn=panel(pos,ttl,accent)
        pn=uipanel('Parent',fig,'Units','normalized','Position',pos, ...
           'BackgroundColor',C.white,'BorderType','line','HighlightColor',C.line);
        uiText(pn,[.035 .88 .95 .11],ttl,8.7,accent,'bold');
    end
    function pn=axesPanel(pos)
        pn=uipanel('Parent',fig,'Units','normalized','Position',pos, ...
           'BackgroundColor',C.white,'BorderType','line','HighlightColor',C.line);
    end
    function h=uiText(parent,pos,str,size,color,weight,alignment)
        if nargin<7,alignment='left';end
        if isequal(parent,fig),bg=C.bg;else,bg=get(parent,'BackgroundColor');end
        h=uicontrol('Parent',parent,'Style','text','Units','normalized', ...
          'Position',pos,'String',str,'FontSize',size,'FontWeight',weight, ...
          'ForegroundColor',color,'BackgroundColor',bg, ...
          'HorizontalAlignment',alignment);
    end
    function h=edit(parent,pos,val)
        h=uicontrol('Parent',parent,'Style','edit','Units','normalized', ...
          'Position',pos,'String',val,'FontSize',9,'HorizontalAlignment','center', ...
          'BackgroundColor','w','ForegroundColor',C.ink);
    end
    function h=smallField(parent,col,row,label,val)
        x=.035+.49*col;
        uiText(parent,[x row .245 .16],label,7.7,C.ink,'normal');
        h=edit(parent,[x+.255 row-.009 .198 .19],val);
    end
    function h=push(parent,pos,label,bg,fg,callback)
        h=uicontrol('Parent',parent,'Style','pushbutton','Units','normalized', ...
         'Position',pos,'String',label,'BackgroundColor',bg,'ForegroundColor',fg, ...
         'FontWeight','bold','FontSize',9,'Callback',callback);
    end
    end
end
