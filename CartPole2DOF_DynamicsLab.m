function CartPole2DOF_DynamicsLab
% INVERTEDPENDULUMSWINGUP_GUI
% SINGLE-RIGID-ROD CART-POLE: swing-up, then PD versus LQR balance.
% Run: CartPole2DOF_DynamicsLab
%
% Single window, same five-card design and side-by-side visual layout as the
% preceding one-rod PD / LQR GUI. No tabs or Simulink required.
% Force charts show swing-up in burgundy, balance in controller color,
% and dashed vertical lines at actual control-mode transitions.
% Energy chart: pendulum energy versus time, target level, actual switching.
% Enlarged window adds a full third row without shrinking earlier charts.
% No Control System Toolbox is needed: PD uses coefficient matching and
% LQR uses the stabilizing Hamiltonian invariant subspace.
%
% Two degrees of freedom: x (cart translation), theta (absolute rod angle
% measured clockwise from upright). theta=0 is upright; +/-pi is down.
% The rigid uniform rod has lc=L/2 and J=m*L^2/3 about the pivot.
% NONLINEAR MODEL:
%  (M+m)*xdd + m*lc*cos(theta)*thetadd - m*lc*omega^2*sin(theta) = u+d
%  m*lc*cos(theta)*xdd + J*thetadd - m*g*lc*sin(theta) = 0
%
% Swing-up: energy-based CART ACCELERATION command. Convert acceleration
% command to cart force using the exact nonlinear equations. Switch to
% -K*[x;xdot;wrap(theta);omega] inside the capture region.
% Exactly downward at rest requires an excitation. The short sinusoidal
% startup kick provides it. Forces are saturated, so successful capture
% depends on the plant and settings. Swing-up assumes sufficient clearance
% BELOW the cart for an ideal rod, as in standard textbook simulations.

    col.bg=[.950 .963 .980]; col.white=[1 1 1];
    col.ink=[.105 .156 .245]; col.muted=[.39 .46 .55];
    col.pd=[.18 .40 .77]; col.lqr=[.04 .57 .51];
    col.line=[.85 .89 .93]; col.orange=[.82 .49 .14];
    col.trail=[.51 .075 .23];
    controllerColors=[col.pd;col.lqr];
    fld=struct(); ui=struct(); saved=[];
    runToken=0; playing=false; isPaused=false;

    screen=get(0,'ScreenSize');
    w=min(1660,max(1120,screen(3)-85));
    baseH=min(1000,max(780,screen(4)-95));
    % Original layout is preserved in PIXELS. Add a chart-height extension
    % below the existing figure area instead of compressing any panels.
    extraH=ceil(.166*baseH);
    extraLabH=58; % Extra experimental controls, WITHOUT shrinking any chart.
    h=baseH+extraH+extraLabH;
    fig=figure('Name','CART-POLE DYNAMICS LAB | SWING-UP + PD vs LQR', ...
        'NumberTitle','off','MenuBar','none','ToolBar','none', ...
        'Color',col.bg,'Units','pixels', ...
        'Position',[max(15,(screen(3)-w)/2) max(30,(screen(4)-h)/2) w h], ...
        'Resize','on','CloseRequestFcn',@onClose);

    % -------------------- HEADER: SAME DESIGN -------------------------
    textUI(fig,[.016 .947 .68 .038], ...
        'CART-POLE  /  CONTROL LAB',18,col.ink,'bold');
    textUI(fig,[.016 .927 .72 .020], ...
        'ONE RIGID ROD  |  NONLINEAR SWING-UP  |  PD vs LQR  |  IDENTICAL START', ...
        9,col.muted,'normal');
    textUI(fig,[.745 .952 .24 .024], ...
        'TWO ANIMATIONS + SIX LIVE CHARTS',8.5,col.muted,'bold','right');

    % -------------------- FIVE ORIGINAL PARAMETER CARDS --------------
    cy=.730;ch=.192;
    plant=card([.014 cy .220 ch],'01  /  PLANT + INITIAL STATE',col.pd);
    cfg=card([.241 cy .160 ch],'02  /  RUN + DISTURBANCE',col.pd);
    pdPole=card([.408 cy .181 ch],'03  /  PD POLES',col.pd);
    pdGain=card([.596 cy .187 ch],'04  /  PD GAINS',col.pd);
    lqr=card([.790 cy .196 ch],'05  /  LQR WEIGHTS',col.lqr);

    fld.M=mini(plant,0,.72,'Cart M','1');
    fld.m=mini(plant,1,.72,'Rod m','0.2');
    fld.L=mini(plant,0,.51,'Length L','0.5');
    fld.g=mini(plant,1,.51,'g','9.81');
    fld.x0=mini(plant,0,.30,'x0 (m)','0');
    fld.v0=mini(plant,1,.30,'v0 (m/s)','0');
    fld.th0=mini(plant,0,.09,'th0 (deg)','180');
    fld.w0=mini(plant,1,.09,'w0 deg/s','0');

    fld.duration=mini(cfg,0,.72,'Time s','8');
    fld.maxF=mini(cfg,1,.72,'Max N','25');
    fld.pulse=mini(cfg,0,.51,'Pulse N','0');
    fld.pStart=mini(cfg,1,.51,'At s','1');
    fld.pDur=mini(cfg,0,.30,'Width s','0.15');
    fld.speed=mini(cfg,1,.30,'Speed','1');
    fld.trail=mini(cfg,0,.09,'Dots s','0.5');
    textUI(cfg,[.57 .09 .40 .17],'Tip trail',8,col.muted,'normal','center');

    textUI(pdPole,[.06 .79 .17 .13],'Pole',8,col.muted,'bold');
    textUI(pdPole,[.32 .79 .27 .13],'Real',8,col.muted,'bold','center');
    textUI(pdPole,[.64 .79 .30 .13],'Imag.',8,col.muted,'bold','center');
    fld.pr=cell(1,4);fld.pi=cell(1,4);
    for n=1:4
        yy=.645-(n-1)*.168;
        textUI(pdPole,[.06 yy .20 .14],sprintf('s%d',n),9,col.ink,'bold');
        if n==3
            realDefault='-0.5';imagDefault='4';
        elseif n==4
            realDefault='-0.5';imagDefault='-4';
        else
            realDefault=num2str(-1-n);imagDefault='0';
        end
        fld.pr{n}=editUI(pdPole,[.28 yy .33 .17],realDefault);
        fld.pi{n}=editUI(pdPole,[.64 yy .30 .17],imagDefault);
    end

    gainLabels={'Kx','Kdx','Ktheta','Kdtheta'};
    fld.k=cell(1,4);
    for n=1:4
        yy=.72-(n-1)*.173;
        textUI(pdGain,[.055 yy .40 .16],gainLabels{n},8.5,col.ink,'bold');
        fld.k{n}=editUI(pdGain,[.49 yy-.005 .45 .18],'0');
    end
    ui.autotune=button(pdGain,[.052 .017 .57 .155], ...
        'AUTO-TUNE PD',col.pd,col.white,@onTune,8);
    fld.manual=uicontrol('Parent',pdGain,'Style','checkbox','Units','normalized', ...
        'Position',[.65 .018 .32 .14],'String','Manual K','Value',0, ...
        'FontSize',8,'BackgroundColor',col.white,'ForegroundColor',col.ink);

    fld.qx=mini(lqr,0,.70,'Q: x','10');
    fld.qv=mini(lqr,1,.70,'Q: dx','1');
    fld.qt=mini(lqr,0,.48,'Q: theta','100');
    fld.qw=mini(lqr,1,.48,'Q: dtheta','1');
    fld.R=mini(lqr,0,.26,'R: force','1');
    textUI(lqr,[.54 .28 .44 .15],'K via Riccati',8,col.muted,'normal','center');
    ui.lqrPill=textUI(lqr,[.06 .025 .91 .16], ...
        'K calculated from Q and R',8,col.lqr,'normal','center');

    % ------------- SLIM SWING-UP STRIP: NO TABS, SAME WINDOW -----------
    sw=uipanel('Parent',fig,'Units','normalized', ...
        'Position',lift([.014 .689 .972 .035]),'BackgroundColor',col.white, ...
        'BorderType','line','HighlightColor',col.line);
    fld.swing=uicontrol('Parent',sw,'Style','checkbox','Units','normalized', ...
        'Position',[.012 .14 .116 .69], ...
        'String','SWING-UP ON','Value',1,'FontWeight','bold','FontSize',8.2, ...
        'ForegroundColor',col.trail,'BackgroundColor',col.white);
    fld.kE=compact(sw,.130,'Energy kE','2',.112);
    fld.kx=compact(sw,.245,'Recenter','2',.110);
    fld.kv=compact(sw,.361,'Cart brake','2',.110);
    fld.kick=compact(sw,.479,'Start kick','2',.110);
    fld.capture=compact(sw,.597,'Capture deg','20',.121);
    fld.captureW=compact(sw,.724,'Max rate','3',.106);
    textUI(sw,[.845 .16 .15 .70], ...
        'Switch to PD / LQR near top',7.6,col.muted,'normal','center');

    % ---- ADDITIONAL PHYSICAL PARAMETERS AND EXPERIMENTS, NO TABS ----
    expY=(.645*baseH+extraH+4)/h;
    expBar=uipanel('Parent',fig,'Units','normalized', ...
        'Position',[.014 expY .972 47/h], ...
        'BackgroundColor',col.white,'BorderType','line', ...
        'HighlightColor',col.line);
    textUI(expBar,[.010 .19 .115 .61],'DYNAMICS STUDIES',8,col.pd,'bold');
    fld.frac=compact(expBar,.129,'COM / L','0.5',.119);
    fld.ig=compact(expBar,.255,'IG / (mL2)','0.0833333',.149);
    fld.linear=uicontrol('Parent',expBar,'Style','checkbox','Units','normalized', ...
        'Position',[.410 .18 .165 .64],'String','Linear overlay', ...
        'FontSize',8.2,'Value',1,'BackgroundColor',col.white, ...
        'ForegroundColor',col.ink);
    ui.sweep=button(expBar,[.575 .13 .145 .74], ...
        'LENGTH SWEEP',col.pd,col.white,@onSweep,8);
    ui.observer=button(expBar,[.730 .13 .119 .74], ...
        'OBSERVER',col.pd,col.white,@onObserver,8);
    ui.export=button(expBar,[.860 .13 .126 .74], ...
        'EXPORT DATA',col.lqr,col.white,@onExport,8);

    % -------------------- PLAYBACK TOOLBAR ----------------------------
    ui.go=button(fig,[.015 .648 .145 .033], ...
        'SIMULATE BOTH',col.ink,col.white,@onRun,10);
    ui.pause=button(fig,[.167 .648 .077 .033], ...
        'PAUSE',[.83 .88 .94],col.ink,@onPause,9);
    ui.reset=button(fig,[.251 .648 .077 .033], ...
        'RESET',[.83 .88 .94],col.ink,@onReset,9);
    ui.status=textUI(fig,[.34 .651 .65 .024], ...
        'Ready. Start at theta=180 deg and click SIMULATE BOTH.', ...
        8.7,col.muted,'normal');

    % -------------------- TWO SAME-SHAPE COMPARISON COLUMNS -----------
    lefts=[.014 .506];width=.478;
    headers={'PD  /  SWING-UP + POLE-PLACEMENT', ...
             'LQR  /  SWING-UP + OPTIMAL FEEDBACK'};
    for j=1:2
        xx=lefts(j);cc=controllerColors(j,:);
        strip=uipanel('Parent',fig,'Units','normalized', ...
            'Position',lift([xx .610 width .032]),'BorderType','none','BackgroundColor',cc);
        uicontrol('Parent',strip,'Style','text','Units','normalized', ...
            'Position',[.015 .11 .50 .79],'String',headers{j}, ...
            'FontSize',9,'FontWeight','bold','ForegroundColor',col.white, ...
            'BackgroundColor',cc,'HorizontalAlignment','left');
        ui.info(j)=uicontrol('Parent',strip,'Style','text','Units','normalized', ...
            'Position',[.49 .11 .50 .79],'String','Ready | SWING-UP', ...
            'FontSize',8,'ForegroundColor',col.white, ...
            'BackgroundColor',cc,'HorizontalAlignment','right');

        panelAnim=axesCard([xx .352 width .251]);
        ui.anim(j)=axes('Parent',panelAnim,'Units','normalized', ...
            'Position',[.10 .16 .865 .79],'Color','w','Box','off', ...
            'XColor',col.ink,'YColor',col.ink,'FontSize',8.5);
        grid(ui.anim(j),'on');hold(ui.anim(j),'on');
        xlabel(ui.anim(j),'Horizontal position, x (m)');
        ylabel(ui.anim(j),'Height relative to rail, y (m)');

        chartY=[.186 .020];chartTitle={'PENDULUM ANGLE','CONTROL FORCE'};
        label={'Angle, theta (deg)','Control force, F (N)'};
        for q=1:2
            ap=axesCard([xx chartY(q) width .158]);
            textUI(ap,[.016 .89 .39 .10],chartTitle{q},8.5,cc,'bold');
            if q==2
                % Small colored phase labels in the existing card header.
                % No extra axes, panels, or layout changes are needed.
                textUI(ap,[.395 .89 .26 .10],'SWING-UP',7.7, ...
                    col.trail,'bold','right');
                if j==1
                    nearLabel='PD BALANCE';
                else
                    nearLabel='LQR BALANCE';
                end
                textUI(ap,[.675 .89 .30 .10],nearLabel,7.7, ...
                    cc,'bold','right');
            end
            ax=axes('Parent',ap,'Units','normalized', ...
                'Position',[.10 .26 .864 .65],'Color','w','Box','off', ...
                'XColor',col.ink,'YColor',col.ink,'FontSize',8);
            grid(ax,'on');hold(ax,'on');set(ax,'GridAlpha',.12);
            xlabel(ax,'Time (s)');ylabel(ax,label{q});
            ui.graph(j,q)=ax;
        end

        % New full-height chart in the EXTRA window region, below force.
        enPanel=axesCard([xx .020 width .158],true);
        textUI(enPanel,[.016 .89 .41 .10],'PENDULUM ENERGY',8.5,cc,'bold');
        textUI(enPanel,[.61 .89 .35 .10],'TARGET E_d',7.7, ...
            col.orange,'bold','right');
        enAx=axes('Parent',enPanel,'Units','normalized', ...
            'Position',[.10 .26 .864 .65],'Color','w','Box','off', ...
            'XColor',col.ink,'YColor',col.ink,'FontSize',8);
        grid(enAx,'on');hold(enAx,'on');set(enAx,'GridAlpha',.12);
        xlabel(enAx,'Time (s)');ylabel(enAx,'Pendulum energy, E (J)');
        ui.graph(j,3)=enAx;
    end

    % Initial view is constructed at launch, without any Simulate click.
    onReset([],[]);

    % -------------------- SMALL REUSABLE GUI HELPERS ------------------
    function pos=lift(pos)
        % New experiment strip sits between toolbar and controller headers.
        % Controls above it shift up. Existing animations keep their pixel size.
        if pos(2)>=.645
            pos(2)=(pos(2)*baseH+extraH+extraLabH)/h;
        else
            pos(2)=(pos(2)*baseH+extraH)/h;
        end
        pos(4)=pos(4)*baseH/h;
    end

    function pos=bottomRow(pos)
        % Energy charts remain below the existing force plots.
        pos(2)=pos(2)*baseH/h;
        pos(4)=pos(4)*baseH/h;
    end

    function pa=card(pos,label,accent)
        pa=uipanel('Parent',fig,'Units','normalized','Position',lift(pos), ...
            'BackgroundColor',col.white,'BorderType','line', ...
            'HighlightColor',col.line,'ShadowColor',col.line);
        textUI(pa,[.04 .884 .95 .10],label,9,accent,'bold');
    end

    function pa=axesCard(pos,isEnergy)
        if nargin>1 && isEnergy
            pos=bottomRow(pos);
        else
            pos=lift(pos);
        end
        pa=uipanel('Parent',fig,'Units','normalized','Position',pos, ...
            'BackgroundColor',col.white,'BorderType','line', ...
            'HighlightColor',col.line,'ShadowColor',col.line);
    end

    function h=textUI(parent,pos,str,size,fg,weight,align)
        if nargin<7,align='left';end
        if strcmp(get(parent,'Type'),'figure')
            pos=lift(pos);
            back=get(parent,'Color');
        else
            back=get(parent,'BackgroundColor');
        end
        h=uicontrol('Parent',parent,'Style','text','Units','normalized', ...
            'Position',pos,'String',str,'FontSize',size, ...
            'FontWeight',weight,'ForegroundColor',fg, ...
            'BackgroundColor',back,'HorizontalAlignment',align);
    end

    function h=editUI(parent,pos,value)
        h=uicontrol('Parent',parent,'Style','edit','Units','normalized', ...
            'Position',pos,'String',value,'FontSize',9, ...
            'ForegroundColor',col.ink,'BackgroundColor',col.white, ...
            'HorizontalAlignment','center');
    end

    function h=mini(parent,side,y,label,value)
        base=.045+side*.49;
        textUI(parent,[base y .255 .155],label,8,col.ink,'normal');
        h=editUI(parent,[base+.260 y-.006 .190 .180],value);
    end

    function h=compact(parent,x,label,value,ww)
        % Label and edit field share one consistent mini-grid column.
        textUI(parent,[x .17 ww*.67 .68],label,7.8,col.ink,'normal');
        h=editUI(parent,[x+ww*.66 .13 ww*.34 .75],value);
    end

    function h=button(parent,pos,str,bg,fg,callback,size)
        if isequal(parent,fig),pos=lift(pos);end
        h=uicontrol('Parent',parent,'Style','pushbutton','Units','normalized', ...
            'Position',pos,'String',str,'FontWeight','bold', ...
            'FontSize',size,'ForegroundColor',fg,'BackgroundColor',bg, ...
            'Callback',callback);
    end

    % -------------------- MODEL AND CONTROL DESIGN --------------------
    function v=num(h,label)
        v=str2double(get(h,'String'));
        if ~isscalar(v)||~isfinite(v)
            error('Please enter a finite number for %s.',label);
        end
    end

    function p=inputs()
        p.M=num(fld.M,'cart mass');p.m=num(fld.m,'rod mass');
        p.L=num(fld.L,'rod length');p.g=num(fld.g,'gravity');
        p.z0=[num(fld.x0,'initial x');num(fld.v0,'initial velocity'); ...
              num(fld.th0,'initial angle')*pi/180; ...
              num(fld.w0,'initial angular velocity')*pi/180];
        p.T=num(fld.duration,'duration');p.maxF=num(fld.maxF,'max force');
        p.pulse=num(fld.pulse,'disturbance force');
        p.tPulse=num(fld.pStart,'disturbance start');
        p.dtPulse=num(fld.pDur,'disturbance width');
        p.speed=num(fld.speed,'playback speed');
        p.trail=num(fld.trail,'tip trail duration');
        p.swing=logical(get(fld.swing,'Value'));
        p.kE=num(fld.kE,'swing-up energy gain');
        p.kx=num(fld.kx,'swing-up cart recenter gain');
        p.kv=num(fld.kv,'swing-up cart velocity gain');
        p.kick=num(fld.kick,'startup excitation');
        p.capture=num(fld.capture,'capture angle')*pi/180;
        p.captureW=num(fld.captureW,'capture angular speed');
        p.Q=diag([num(fld.qx,'Qx'),num(fld.qv,'Q velocity'), ...
                  num(fld.qt,'Q angle'),num(fld.qw,'Q angular rate')]);
        p.R=num(fld.R,'R');
        if any([p.M,p.m,p.L,p.g]<=0)
            error('M, m, L, and g must be strictly positive.');
        end
        if p.T<=0||p.T>60
            error('Simulation duration must be greater than zero and at most 60 s.');
        end
        if p.maxF<=0,error('Max force must be positive.');end
        if p.speed<=0||p.speed>20,error('Playback speed must be between 0 and 20.');end
        if any([p.trail p.tPulse p.dtPulse]<0)
            error('Trail, disturbance start and duration must be nonnegative.');
        end
        if p.kE<0||p.kx<0||p.kv<0
            error('Swing-up gains must be nonnegative.');
        end
        if p.capture<=0||p.capture>=pi/2||p.captureW<=0
            error('Capture angle must be between 0 and 90 deg; max rate must be positive.');
        end
        if p.R<=0||any(diag(p.Q)<0)
            error('LQR requires R > 0 and all Q state weights >= 0.');
        end
        p.frac=num(fld.frac,'COM position / L');
        p.ig=num(fld.ig,'IG / (mL^2)');
        if p.frac<=0 || p.frac>=1 || p.ig<=0
            error('COM/L must be between 0 and 1; IG/(mL2) must be positive.');
        end
        p=buildPlant(p);
    end

    function p=buildPlant(p)
        % Arbitrary rigid-body COM and transverse mass moment of inertia.
        p.lc=p.frac*p.L;
        p.IG=p.ig*p.m*p.L^2;
        p.J=p.IG+p.m*p.lc^2;
        p.a=p.M+p.m;p.b=p.m*p.lc;p.k=p.m*p.g*p.lc;
        p.D=p.a*p.J-p.b*p.b;p.Ed=2*p.k;
        if p.D<=0,error('Nonpositive rigid-body mass determinant.');end
        p.A=[0 1 0 0;0 0 -p.b*p.k/p.D 0; ...
             0 0 0 1;0 0 p.a*p.k/p.D 0];
        p.B=[0;p.J/p.D;0;-p.b/p.D];
    end

    function K=pdGains(p,updateFields)
        if nargin<2,updateFields=true;end
        if get(fld.manual,'Value')==1
            K=zeros(1,4);
            for ii=1:4,K(ii)=num(fld.k{ii},sprintf('manual gain %d',ii));end
            return
        end
        poles=complex(zeros(1,4));
        for ii=1:4
            poles(ii)=complex(num(fld.pr{ii},'pole real'), ...
                              num(fld.pi{ii},'pole imaginary'));
        end
        if any(real(poles)>=0)
            error('Enter stable PD poles with negative real parts.');
        end
        q=sortrows([real(poles(:)),imag(poles(:))],[1 2]);
        qc=sortrows([real(conj(poles(:))),imag(conj(poles(:)))],[1 2]);
        if max(abs(complex(q(:,1),q(:,2))-complex(qc(:,1),qc(:,2))))>1e-7
            error('Complex poles must occur as conjugate pairs.');
        end
        c=real(poly(poles));
        % Exact characteristic coefficient matching for K=[Kx Kdx Kth Kdth].
        kx=-p.D*c(5)/p.k;
        kdx=-p.D*c(4)/p.k;
        kth=(p.J*kx-p.a*p.k-p.D*c(3))/p.b;
        kdth=(p.J*kdx-p.D*c(2))/p.b;
        K=[kx kdx kth kdth];
        if any(~isfinite(K)),error('PD gain calculation was not finite.');end
        if updateFields
            for ii=1:4,set(fld.k{ii},'String',sprintf('%.7g',K(ii)));end
        end
    end

    function K=lqrGains(p)
        % Stabilizing CARE solution without needing the Control System Toolbox.
        A=p.A;B=p.B;Q=p.Q;R=p.R;
        H=[A,-B*B'/R;-Q,-A'];
        [V,EV]=eig(H);
        ix=find(real(diag(EV))<-1e-9);
        if numel(ix)~=4
            error('LQR has no stable Riccati subspace; try positive Q weights.');
        end
        X=V(1:4,ix);Y=V(5:8,ix);
        if rcond(X)<1e-11,error('Ill-conditioned LQR matrix: adjust Q and R.');end
        P=real(Y/X);P=(P+P')/2;
        residual=A'*P+P*A-P*(B*B')*P/R+Q;
        if norm(residual,'fro')>1e-6*max(1,norm(Q,'fro')+norm(P,'fro')*norm(A,'fro'))
            error('Riccati solution residual too large. Try different LQR weights.');
        end
        K=real((B'*P)/R);
        if any(real(eig(A-B*K))>=0)
            error('The LQR weights do not yield stable closed-loop poles.');
        end
    end

    function ang=wrapAngle(theta)
        ang=atan2(sin(theta),cos(theta));
    end

    function d=disturbance(t,p)
        d=0;
        if p.dtPulse>0 && t>=p.tPulse && t<p.tPulse+p.dtPulse
            d=p.pulse;
        end
    end

    function [F,acmd,E]=control(t,z,p,K,mode)
        % mode=0 for energy swing-up, 1 for linear upright stabilization.
        x=z(1);xd=z(2);th=z(3);om=z(4);
        E=.5*p.J*om^2+p.k*(1+cos(th));
        if mode==0
            errorEnergy=p.Ed-E;
            % Startup excitation breaks exact bottom-equilibrium symmetry.
            excitation=0;
            if t<.8,excitation=p.kick*sin(2*pi*.8*t);end
            acmd=-p.kE*errorEnergy*om*cos(th) ...
                 -p.kx*x-p.kv*xd+excitation;
            % Limit requested acceleration before converting it to force.
            aMax=max(1,p.maxF/p.a*.65);
            acmd=max(-aMax,min(aMax,acmd));
            % Nonlinear inverse dynamics converting xdd command into FORCE.
            F=(p.a-(p.b^2/p.J)*cos(th)^2)*acmd ...
              +(p.b*p.k/p.J)*sin(th)*cos(th) ...
              -p.b*om^2*sin(th);
        else
            acmd=NaN;
            F=-K*[x;xd;wrapAngle(th);om];
        end
        F=max(-p.maxF,min(p.maxF,F));
    end

    function dz=dynamics(t,z,p,K,mode)
        [F,~,~]=control(t,z,p,K,mode);
        th=z(3);om=z(4);bc=p.b*cos(th);
        detM=p.a*p.J-bc^2;
        rhs=[F+disturbance(t,p)+p.b*om^2*sin(th);p.k*sin(th)];
        dd=[p.J -bc;-bc p.a]*rhs/detM;
        dz=[z(2);dd(1);om;dd(2)];
    end

    function [t,Z,U,DD,phase,captureTime,endedEarly]=simulate(p,K)
        % Fixed output steps + classical RK4. Switching only at accepted
        % time-grid boundaries avoids solver-stage-dependent hybrid modes.
        n=max(2,ceil(p.T/.01)+1);
        t=linspace(0,p.T,n).';dt=t(2)-t(1);
        Z=zeros(n,4);U=zeros(n,1);DD=zeros(n,1);phase=zeros(n,1);
        Z(1,:)=p.z0.';
        mode=double(~p.swing);captureTime=NaN;endedEarly=false;
        last=n;
        for i=1:n
            zi=Z(i,:).';ang=wrapAngle(zi(3));
            if p.swing
                if mode==0 && abs(ang)<=p.capture && ...
                        abs(zi(4))<=p.captureW && abs(zi(1))<max(4*p.L,1)
                    mode=1;
                    if isnan(captureTime),captureTime=t(i);end
                elseif mode==1 && abs(ang)>max(.85,2.2*p.capture)
                    mode=0;
                end
            end
            phase(i)=mode;
            U(i)=control(t(i),zi,p,K,mode);
            DD(i)=disturbance(t(i),p);
            if i==n,break;end
            h=dt;ti=t(i);
            k1=dynamics(ti,zi,p,K,mode);
            k2=dynamics(ti+h/2,zi+h*k1/2,p,K,mode);
            k3=dynamics(ti+h/2,zi+h*k2/2,p,K,mode);
            k4=dynamics(ti+h,zi+h*k3,p,K,mode);
            next=zi+h*(k1+2*k2+2*k3+k4)/6;
            if any(~isfinite(next)) || abs(next(1))>30 || ...
               abs(next(2))>100 || abs(next(4))>100
                endedEarly=true;last=i;break;
            end
            Z(i+1,:)=next.';
        end
        t=t(1:last);Z=Z(1:last,:);U=U(1:last);
        DD=DD(1:last);phase=phase(1:last);
    end

    function onTune(~,~)
        try
            % Button deliberately turns manual override off.
            set(fld.manual,'Value',0);
            p=inputs();K=pdGains(p,true);
            set(ui.status,'String',sprintf( ...
                'PD auto-tuned: K=[%.3g %.3g %.3g %.3g]. Click SIMULATE BOTH.',K));
        catch err
            set(ui.status,'String',err.message);
            errordlg(err.message,'PD auto-tune');
        end
    end

    % -------------------- RUN, RESET, PAUSE ----------------------------
    function onRun(~,~)
        runToken=runToken+1;token=runToken;
        playing=false;isPaused=false;set(ui.pause,'String','PAUSE');
        try
            p=inputs();Kpd=pdGains(p,true);Klqr=lqrGains(p);
            p.Kpd=Kpd;p.Klqr=Klqr;
            set(ui.lqrPill,'String',sprintf( ...
                'K=[%.2f %.2f %.2f %.2f]',Klqr));
            set(ui.status,'String','Calculating nonlinear swing-up and capture trajectories...');
            drawnow;
            [t1,Z1,F1,D1,H1,C1,E1]=simulate(p,Kpd);
            [t2,Z2,F2,D2,H2,C2,E2]=simulate(p,Klqr);
            n=min(numel(t1),numel(t2));
            t=t1(1:n);ZZ={Z1(1:n,:),Z2(1:n,:)};
            FF={F1(1:n),F2(1:n)};PH={H1(1:n),H2(1:n)};
            DD=D1(1:n);
            saved=struct('p',p,'t',t,'ZZ',{ZZ},'FF',{FF}, ...
                'DD',DD,'PH',{PH},'capture',[C1 C2],'early',E1||E2);
            saved.LZ={[],[]};
            if logical(get(fld.linear,'Value')) && ~p.swing
                % Evaluate the linearized plant using the same state gains.
                % Enabled only when swing-up is off, since a model expanded
                % around upright is not meaningful from theta=180 degrees.
                saved.LZ{1}=linearTrial(p,Kpd,t);
                saved.LZ{2}=linearTrial(p,Klqr,t);
            end
            preparePlots(saved);
            if ~ishandle(fig)||token~=runToken,return;end
            playing=true;
            set(ui.status,'String', ...
                'Simultaneous playback: identical swing-up laws, different capture feedback.');
            animate(token);
        catch err
            if ishandle(fig)
                set(ui.status,'String',err.message);
                errordlg(err.message,'Simulation error');
            end
        end
    end

    function Z=linearTrial(p,K,t)
        [~,Z]=ode45(@(tt,z) p.A*z+p.B*( ...
            max(-p.maxF,min(p.maxF,-K*z))+disturbance(tt,p)), ...
            t,p.z0,odeset('RelTol',1e-8,'AbsTol',1e-10));
    end

    function onSweep(~,~)
        try
            p=inputs();p.swing=false;
            if abs(p.z0(3))>pi/3
                p.z0(3)=5*pi/180;
            end
            Kp=pdGains(p,false);Kl=lqrGains(p);
            scales=[.70 1 1.30];colors=lines(numel(scales));
            ff=figure('Name','Chapter 8: rod-length sensitivity, fixed gains', ...
                'NumberTitle','off','Color','w','Position',[140 90 1040 660]);
            for j=1:2
                if j==1,K=Kp;name='Pole placement';
                else,K=Kl;name='LQR';end
                aa=subplot(2,2,j,'Parent',ff);hold(aa,'on');grid(aa,'on');
                bb=subplot(2,2,j+2,'Parent',ff);hold(bb,'on');grid(bb,'on');
                for ii=1:numel(scales)
                    pp=p;pp.L=p.L*scales(ii);pp=buildPlant(pp);
                    [tt,zz,uu]=simulate(pp,K);
                    plot(aa,tt,zz(:,3)*180/pi,'LineWidth',1.7, ...
                         'Color',colors(ii,:));
                    plot(bb,tt,uu,'LineWidth',1.4,'Color',colors(ii,:));
                end
                title(aa,[name ' | angle']);xlabel(aa,'Time (s)');ylabel(aa,'Angle (deg)');
                title(bb,[name ' | force']);xlabel(bb,'Time (s)');ylabel(bb,'Force (N)');
                legend(aa,{'0.7 L','1.0 L','1.3 L'},'Location','best');
            end
            annotation(ff,'textbox',[.1 .96 .8 .035], ...
                'String','Length changes, but controller gains remain fixed at nominal values', ...
                'EdgeColor','none','HorizontalAlignment','center');
        catch err
            errordlg(err.message,'Parameter sweep');
        end
    end

    function onObserver(~,~)
        % Chapter 4 observer demo using two position measurements x, theta.
        % A linear Kalman-Bucy style observer reconstructs both velocities.
        try
            if isempty(saved)
                error('Run a near-upright simulation before the observer demo.');
            end
            p=saved.p;
            if p.swing || max(abs([saved.ZZ{1}(:,3);saved.ZZ{2}(:,3)]))>pi/6
                error('Observer demo requires swing-up OFF and tilt within 30 deg, as it uses an upright linearization.');
            end
            Cobs=[1 0 0 0;0 0 1 0];
            Qproc=diag([.001 .1 .001 .1]);
            Rmeas=diag([.0001 (0.5*pi/180)^2]);
            hh=[p.A' -Cobs'*(Rmeas\Cobs);-Qproc -p.A];
            [V,EV]=eig(hh);
            idx=find(real(diag(EV))< -1e-8);
            if numel(idx)~=4,error('Observer Riccati solution did not have four stable modes.');end
            X=V(1:4,idx);Y=V(5:8,idx);
            if rcond(X)<1e-11,error('Observer Riccati problem ill-conditioned.');end
            PP=real(Y/X);PP=(PP+PP')/2;
            Lo=PP*Cobs'/Rmeas;
            tt=saved.t;trueZ=saved.ZZ{1};Ff=saved.FF{1};
            backup=rng;rng(7);
            noisy=[trueZ(:,1),trueZ(:,3)] + ...
                randn(numel(tt),2)*diag(sqrt(diag(Rmeas)));
            rng(backup);
            zh=zeros(numel(tt),4);zh(1,:)=[0 0 0 0];
            for ii=2:numel(tt)
                dt=tt(ii)-tt(ii-1);
                residual=noisy(ii-1,:)'-Cobs*zh(ii-1,:)';
                xdot=p.A*zh(ii-1,:)'+p.B*(Ff(ii-1)+disturbance(tt(ii-1),p)) + Lo*residual;
                zh(ii,:)=zh(ii-1,:)+dt*xdot';
            end
            fig2=figure('Name','Chapter 4: sensing and velocity estimation', ...
                'Color','w','Position',[130 95 1070 680]);
            a1=subplot(2,2,1,'Parent',fig2);
            plot(a1,tt,trueZ(:,1),'LineWidth',1.7);hold(a1,'on');
            plot(a1,tt,noisy(:,1),'.','MarkerSize',2);grid(a1,'on');
            xlabel(a1,'Time (s)');ylabel(a1,'Cart x (m)');
            legend(a1,{'True x','Noisy measured x'},'Location','best');
            a2=subplot(2,2,2,'Parent',fig2);
            plot(a2,tt,trueZ(:,3)*180/pi,'LineWidth',1.7);hold(a2,'on');
            plot(a2,tt,noisy(:,2)*180/pi,'.','MarkerSize',2);grid(a2,'on');
            xlabel(a2,'Time (s)');ylabel(a2,'Tilt (deg)');
            legend(a2,{'True angle','Noisy angle'},'Location','best');
            a3=subplot(2,2,3,'Parent',fig2);
            plot(a3,tt,trueZ(:,2),'LineWidth',1.8);hold(a3,'on');
            plot(a3,tt,zh(:,2),'--','LineWidth',1.6);grid(a3,'on');
            xlabel(a3,'Time (s)');ylabel(a3,'Cart velocity (m/s)');
            legend(a3,{'True','Observer estimate'},'Location','best');
            a4=subplot(2,2,4,'Parent',fig2);
            plot(a4,tt,trueZ(:,4)*180/pi,'LineWidth',1.8);hold(a4,'on');
            plot(a4,tt,zh(:,4)*180/pi,'--','LineWidth',1.6);grid(a4,'on');
            xlabel(a4,'Time (s)');ylabel(a4,'Angular rate (deg/s)');
            legend(a4,{'True','Observer estimate'},'Location','best');
        catch err
            errordlg(err.message,'Observer demonstration');
        end
    end

    function onExport(~,~)
        if isempty(saved),errordlg('Run SIMULATE BOTH before exporting.');return;end
        [fn,fp]=uiputfile('CartPole2DOF_results.mat','Export MATLAB time histories');
        if isequal(fn,0),return;end
        results=saved;save(fullfile(fp,fn),'results');
    end

    function onReset(~,~)
        runToken=runToken+1;playing=false;isPaused=false;
        set(ui.pause,'String','PAUSE');
        try
            p=inputs();Kpd=pdGains(p,true);Klqr=lqrGains(p);
            set(ui.lqrPill,'String',sprintf( ...
                'K=[%.2f %.2f %.2f %.2f]',Klqr));
            phase0=double(~p.swing);
            t=[0;.01];Z=repmat(p.z0.',2,1);
            FF={repmat(control(0,p.z0,p,Kpd,phase0),2,1), ...
                repmat(control(0,p.z0,p,Klqr,phase0),2,1)};
            data=struct('p',p,'t',t,'ZZ',{{Z,Z}},'FF',{FF}, ...
                'DD',zeros(2,1),'PH',{{phase0*ones(2,1), ...
                phase0*ones(2,1)}},'capture',[NaN NaN],'early',false);
            preparePlots(data);
            set(ui.status,'String', ...
                'Ready. Swing-up from 180 deg, then PD on left and LQR on right.');
        catch err
            set(ui.status,'String',err.message);
        end
    end

    function onPause(~,~)
        if ~playing,return;end
        isPaused=~isPaused;
        if isPaused
            set(ui.pause,'String','RESUME');
            set(ui.status,'String','Playback paused for both controllers.');
        else
            set(ui.pause,'String','PAUSE');
            set(ui.status,'String','Playback resumed.');
        end
    end

    function onClose(~,~)
        runToken=runToken+1;playing=false;
        if ishandle(fig),delete(fig);end
    end

    % -------------------- ANIMATION AND THREE CHARTS PER COLUMN -------
    function preparePlots(data)
        p=data.p;t=data.t;ZZ=data.ZZ;FF=data.FF;DD=data.DD;
        cw=max(.5*p.L,.23);
        xx=[ZZ{1}(:,1);ZZ{2}(:,1); ...
            ZZ{1}(:,1)+p.L*sin(ZZ{1}(:,3)); ...
            ZZ{2}(:,1)+p.L*sin(ZZ{2}(:,3));0];
        xmin=min(xx)-cw;xmax=max(xx)+cw;
        mid=(xmin+xmax)/2;span=max(xmax-xmin,4.3*p.L);
        % Wide enough to show total cart travel and equal-scale wheels.
        commonX=[mid-span/2 mid+span/2];
        if p.swing
            % Essential physical visualization detail: the downward rod
            % reaches BELOW the rail. The previous [0,2L] view hides it.
            commonY=[-1.12*p.L 2*p.L];
        else
            commonY=[0 2*p.L];
        end
        aa=[wrapSeries(ZZ{1}(:,3));wrapSeries(ZZ{2}(:,3))]*180/pi;
        yyAngle=limits(aa,5);
        if p.swing
            yyAngle=[-195 195];
        end
        yyForce=limits([FF{1};FF{2};DD],1);
        % This is precisely the energy used by the swing-up law.
        % Relative to the DOWN equilibrium, not total cart energy.
        EE=cell(1,2);
        for jj=1:2
            zEnergy=ZZ{jj};
            EE{jj}=.5*p.J*zEnergy(:,4).^2+ ...
                p.k*(1+cos(zEnergy(:,3)));
        end
        energyMax=max([p.Ed;EE{1};EE{2}]);
        energyLim=[0 max(1.15*p.Ed,energyMax*1.12+1e-9)];
        for j=1:2
            ax=ui.anim(j);cla(ax);hold(ax,'on');grid(ax,'on');
            set(ax,'GridAlpha',.12,'Box','off');
            xlim(ax,commonX);ylim(ax,commonY);
            % Critical: equal x/y scale, so cart wheels remain CIRCULAR.
            daspect(ax,[1 1 1]);
            set(ax,'XLimMode','manual','YLimMode','manual');
            plot(ax,commonX,[0 0],'-','Color',col.muted,'LineWidth',2.1);
            plot(ax,[0 0],commonY,':','Color',col.line,'LineWidth',1);
            xlabel(ax,'Horizontal position, x (m)');
            ylabel(ax,'Height relative to rail, y (m)');
            createCart(j,p,cw,ZZ{j}(1,:));

            for q=1:2
                a=ui.graph(j,q);cla(a);hold(a,'on');grid(a,'on');
                set(a,'GridAlpha',.11,'Box','off');
                xlim(a,[0 max(.5,t(end))]);
                if q==1
                    ylim(a,yyAngle);ylabel(a,'Angle, theta (deg)');
                    plot(a,[0 max(.5,t(end))],[0 0],':', ...
                        'Color',col.muted,'LineWidth',1);
                else
                    ylim(a,yyForce);ylabel(a,'Control force, F (N)');
                end
                xlabel(a,'Time (s)');
                if q==1
                    ui.line(j,q)=plot(a,nan,nan,'-', ...
                        'Color',controllerColors(j,:),'LineWidth',1.8);
                    ui.linearLine(j)=plot(a,nan,nan,'--', ...
                        'Color',[.50 .53 .58],'LineWidth',1.2);
                else
                    % Same force plot, but different line colors by the
                    % actual active controller at each simulation step.
                    ui.forceSwing(j)=plot(a,nan,nan,'-', ...
                        'Color',col.trail,'LineWidth',2.0);
                    ui.forceBalance(j)=plot(a,nan,nan,'-', ...
                        'Color',controllerColors(j,:),'LineWidth',2.0);
                    % A dashed vertical marker is shown at each switch,
                    % including a switch back to swing-up if capture fails.
                    ui.forceSwitch(j)=plot(a,nan,nan,'--', ...
                        'Color',[.43 .47 .54],'LineWidth',1.2);
                    ui.dist(j)=plot(a,nan,nan,':','Color',col.orange, ...
                        'LineWidth',1.0);
                end
                ui.cursor(j,q)=plot(a,[0 0],get(a,'YLim'),':', ...
                    'Color',col.muted,'LineWidth',.8);
            end
            ui.switchLine(j)=plot(ui.graph(j,1),nan,nan,':', ...
                'Color',col.trail,'LineWidth',1.25);

            ea=ui.graph(j,3);cla(ea);hold(ea,'on');grid(ea,'on');
            set(ea,'GridAlpha',.11,'Box','off');
            xlim(ea,[0 max(.5,t(end))]);ylim(ea,energyLim);
            xlabel(ea,'Time (s)');ylabel(ea,'Pendulum energy, E (J)');
            ui.energyTarget(j)=plot(ea,[0 max(.5,t(end))], ...
                [p.Ed p.Ed],'--','Color',col.orange,'LineWidth',1.3);
            ui.energySwing(j)=plot(ea,nan,nan,'-', ...
                'Color',col.trail,'LineWidth',1.8);
            ui.energyBalance(j)=plot(ea,nan,nan,'-', ...
                'Color',controllerColors(j,:),'LineWidth',1.8);
            ui.energySwitch(j)=plot(ea,nan,nan,'--', ...
                'Color',[.43 .47 .54],'LineWidth',1.2);
            ui.cursor(j,3)=plot(ea,[0 0],energyLim,':', ...
                'Color',col.muted,'LineWidth',.8);
        end
        for j=1:2,updateFrame(j,1,data);end
        drawnow;
    end

    function wrapped=wrapSeries(th)
        wrapped=atan2(sin(th),cos(th));
    end

    function lim=limits(vals,margin)
        vals=vals(isfinite(vals));if isempty(vals),vals=0;end
        lo=min([vals(:);0]);hi=max([vals(:);0]);
        extra=max(margin,.15*max(hi-lo,max(abs([lo hi]))));
        lim=[lo-extra hi+extra];
    end

    function createCart(j,p,cw,z)
        ax=ui.anim(j);x=z(1);th=z(3);
        wr=.045*p.L;by=2*wr;bh=.18*p.L;py=by+bh;
        wheelX=.31*cw;tt=linspace(0,2*pi,48);
        tipX=x+p.L*sin(th);tipY=py+p.L*cos(th);
        rod=controllerColors(j,:);
        s.wheel1=patch(ax,x-wheelX+wr*cos(tt),wr+wr*sin(tt), ...
            col.white,'EdgeColor',col.ink,'LineWidth',1.2);
        s.wheel2=patch(ax,x+wheelX+wr*cos(tt),wr+wr*sin(tt), ...
            col.white,'EdgeColor',col.ink,'LineWidth',1.2);
        s.body=patch(ax,x+cw*[-.5 .5 .5 -.5], ...
            by+bh*[0 0 1 1],col.ink,'EdgeColor',col.ink);
        s.rod=plot(ax,[x tipX],[py tipY],'-', ...
            'Color',rod,'LineWidth',5.5);
        s.joint=plot(ax,x,py,'o','MarkerFaceColor','w', ...
            'MarkerEdgeColor',col.ink,'MarkerSize',7,'LineWidth',1.4);
        s.trail=scatter(ax,nan,nan,14,col.trail,'o','filled');
        s.tip=plot(ax,tipX,tipY,'o','MarkerFaceColor','w', ...
            'MarkerEdgeColor',rod,'MarkerSize',9,'LineWidth',1.8);
        s.geom=struct('cw',cw,'wr',wr,'by',by,'bh',bh, ...
            'py',py,'wheelX',wheelX,'tt',tt);
        ui.cart{j}=s;
    end

    function updateFrame(j,i,data)
        p=data.p;t=data.t;Z=data.ZZ{j};F=data.FF{j};
        s=ui.cart{j};geo=s.geom;
        x=Z(i,1);th=Z(i,3);
        tipX=x+p.L*sin(th);tipY=geo.py+p.L*cos(th);
        set(s.wheel1,'XData',x-geo.wheelX+geo.wr*cos(geo.tt));
        set(s.wheel2,'XData',x+geo.wheelX+geo.wr*cos(geo.tt));
        set(s.body,'XData',x+geo.cw*[-.5 .5 .5 -.5]);
        set(s.rod,'XData',[x tipX],'YData',[geo.py tipY]);
        set(s.joint,'XData',x,'YData',geo.py);
        set(s.tip,'XData',tipX,'YData',tipY);
        if p.trail>0
            ids=find(t>=t(i)-p.trail & t<=t(i));
            ids=ids(1:3:end);
            if isempty(ids)||ids(end)~=i,ids=[ids;i];end
            cx=Z(ids,1)+p.L*sin(Z(ids,3));
            cy=geo.py+p.L*cos(Z(ids,3));
            fade=linspace(.20,1,numel(ids)).';
            colors=(1-fade)*[.94 .94 .96]+fade*col.trail;
            set(s.trail,'XData',cx,'YData',cy, ...
                'CData',colors,'SizeData',6+16*fade);
        else
            set(s.trail,'XData',nan,'YData',nan);
        end
        angleDeg=wrapSeries(Z(1:i,3))*180/pi;
        % Do not draw false 360-degree jumps across the wrap boundary.
        jumps=find(abs(diff(angleDeg))>180)+1;
        angleDeg(jumps)=NaN;
        set(ui.line(j,1),'XData',t(1:i),'YData',angleDeg);
        if isfield(data,'LZ') && numel(data.LZ)>=j && ~isempty(data.LZ{j})
            set(ui.linearLine(j),'XData',t(1:i), ...
                'YData',data.LZ{j}(1:i,3)*180/pi);
        end
        % Split the actual force history into the swing-up phase and the
        % PD/LQR near-upright phase, without changing the control law.
        modeHistory=data.PH{j}(1:i);
        fSwing=F(1:i);fSwing(modeHistory~=0)=NaN;
        fBalance=F(1:i);fBalance(modeHistory~=1)=NaN;
        set(ui.forceSwing(j),'XData',t(1:i),'YData',fSwing);
        set(ui.forceBalance(j),'XData',t(1:i),'YData',fBalance);
        energy=.5*p.J*Z(1:i,4).^2+p.k*(1+cos(Z(1:i,3)));
        eSwing=energy;eSwing(modeHistory~=0)=NaN;
        eBalance=energy;eBalance(modeHistory~=1)=NaN;
        set(ui.energySwing(j),'XData',t(1:i),'YData',eSwing);
        set(ui.energyBalance(j),'XData',t(1:i),'YData',eBalance);
        % The dashed switch marker moves into view when the switch occurs.
        switchIdx=find(diff(modeHistory)~=0)+1;
        if isempty(switchIdx)
            set(ui.forceSwitch(j),'XData',nan,'YData',nan);
        else
            swT=t(switchIdx);
            yBounds=get(ui.graph(j,2),'YLim');
            sx=reshape([swT swT nan(numel(swT),1)].',[],1);
            sy=reshape([repmat(yBounds(1),numel(swT),1) ...
                        repmat(yBounds(2),numel(swT),1) ...
                        nan(numel(swT),1)].',[],1);
            set(ui.forceSwitch(j),'XData',sx,'YData',sy);
        end
        % Matching actual control-switch instants on the energy plot.
        if isempty(switchIdx)
            set(ui.energySwitch(j),'XData',nan,'YData',nan);
        else
            eBounds=get(ui.graph(j,3),'YLim');
            ex=reshape([swT swT nan(numel(swT),1)].',[],1);
            ey=reshape([repmat(eBounds(1),numel(swT),1) ...
                        repmat(eBounds(2),numel(swT),1) ...
                        nan(numel(swT),1)].',[],1);
            set(ui.energySwitch(j),'XData',ex,'YData',ey);
        end
        for q=1:3,set(ui.cursor(j,q),'XData',[t(i) t(i)]);end
        set(ui.dist(j),'XData',t(1:i),'YData',data.DD(1:i));
        cap=data.capture(j);
        if isfinite(cap)
            set(ui.switchLine(j),'XData',[cap cap], ...
                'YData',get(ui.graph(j,1),'YLim'));
        else
            set(ui.switchLine(j),'XData',nan,'YData',nan);
        end
        stage='SWING';if data.PH{j}(i)==1,stage='BALANCE';end
        set(ui.info(j),'String',sprintf( ...
            '%s | t=%.2fs  theta=%+.1f deg  x=%+.2fm  F=%+.1fN', ...
            stage,t(i),wrapAngle(th)*180/pi,x,F(i)));
    end

    function animate(token)
        p=saved.p;t=saved.t;
        stride=max(1,round(.025/max(1e-6,t(2)-t(1))));
        frames=unique([1:stride:numel(t),numel(t)]);
        for q=1:numel(frames)
            i=frames(q);
            if ~ishandle(fig)||runToken~=token,return;end
            while isPaused && ishandle(fig) && runToken==token
                drawnow;pause(.04);
            end
            if ~ishandle(fig)||runToken~=token,return;end
            tm=tic;
            updateFrame(1,i,saved);updateFrame(2,i,saved);
            drawnow limitrate;
            if q<numel(frames)
                gap=(t(frames(q+1))-t(i))/p.speed;
                pause(max(0,gap-toc(tm)));
            end
        end
        playing=false;
        if ~ishandle(fig)||runToken~=token,return;end
        if saved.early
            msg='Stopped early because of excessive cart displacement or speed.';
        elseif p.swing && any(isnan(saved.capture))
            msg='Simulation ended without capture in at least one column. Adjust swing gains or duration.';
        else
            msg=sprintf('Complete. PD angle=%+.2f deg; LQR angle=%+.2f deg.', ...
                wrapAngle(saved.ZZ{1}(end,3))*180/pi, ...
                wrapAngle(saved.ZZ{2}(end,3))*180/pi);
        end
        set(ui.status,'String',msg);
    end
end
