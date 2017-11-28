function AddedMassGlider = set_AddedMassGlider2017_09_27(in1,in2,in3)
%SET_ADDEDMASSGLIDER2017_09_27
%    ADDEDMASSGLIDER = SET_ADDEDMASSGLIDER2017_09_27(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 5.10.
%    27-Sep-2017 10:18:08

AddedMJgx = in3(4,:);
AddedMJgy = in3(5,:);
AddedMJgz = in3(6,:);
AddedMmgx = in3(1,:);
AddedMmgy = in3(2,:);
AddedMmgz = in3(3,:);
phif = in1(4,:);
phit = in1(7,:);
psif = in1(6,:);
thetaf = in1(5,:);
thetat = in1(8,:);
t2 = thetaf-thetat;
t3 = cos(phif);
t4 = sin(psif);
t5 = cos(psif);
t6 = sin(phif);
t12 = pi.*(1.0./2.0);
t7 = -t12+thetaf;
t8 = cos(t7);
t9 = cos(t2);
t10 = phif-phit;
t11 = t4.*t6;
t13 = t3.*t5.*t8;
t14 = t11+t13;
t15 = cos(t10);
t16 = t3.*t4;
t22 = t5.*t6.*t8;
t17 = t16-t22;
t18 = sin(t2);
t19 = sin(t7);
t20 = sin(t10);
t21 = t9.*t14.*t20.*8.0;
t23 = t9.*t17.*8.0;
t24 = t5.*t6;
t29 = t3.*t4.*t8;
t25 = t24-t29;
t26 = t3.*t5;
t27 = t4.*t6.*t8;
t28 = t26+t27;
t30 = t9.*t20.*t25.*8.0;
t31 = t4.*t9.*t15.*t19.*8.0;
t32 = t9.*t28.*8.0;
t33 = t4.*t18.*t19.*t20.*8.0;
t34 = t8.*t9.*t15.*8.0;
t35 = t3.*t9.*t19.*t20.*8.0;
t36 = t6.*t9.*t19.*8.0;
t37 = t3.*t15.*t18.*t19.*8.0;
t38 = t14.*t18.*4.0;
t39 = t9.*t14.*t20.*4.0;
t51 = t9.*t15.*t17.*4.0;
t52 = t5.*t9.*t15.*t19.*4.0;
t40 = t38+t39-t51-t52;
t41 = t18.*t25.*4.0;
t42 = t9.*t20.*t25.*4.0;
t43 = t4.*t9.*t15.*t19.*4.0;
t50 = t9.*t15.*t28.*4.0;
t44 = t41+t42+t43-t50;
t45 = t3.*t18.*t19.*4.0;
t46 = t8.*t9.*t15.*4.0;
t47 = t6.*t9.*t15.*t19.*4.0;
t48 = t3.*t9.*t19.*t20.*4.0;
t49 = t45+t46+t47+t48;
t53 = t9.*t17.*4.0;
t54 = t6.*t9.*t19.*4.0;
t55 = t3.*t15.*t18.*t19.*4.0;
t56 = t9.*t28.*4.0;
t57 = t4.*t18.*t19.*t20.*4.0;
t58 = t14.*t15.*t18.*4.0;
t59 = t5.*t6.*t18.*t19.*4.0;
t60 = t5.*t8.*t9.*t20.*4.0;
t61 = t5.*t18.*t19.*t20.*4.0;
t62 = t3.*t4.*t9.*t15.*t19.*4.0;
t66 = t15.*t18.*t25.*4.0;
t67 = t4.*t6.*t18.*t19.*4.0;
t68 = t4.*t8.*t9.*t20.*4.0;
t63 = t56+t57+t62-t66-t67-t68;
t64 = t6.*t8.*t18.*4.0;
t69 = t9.*t19.*t20.*4.0;
t70 = t8.*t18.*t20.*4.0;
t71 = t3.*t8.*t9.*t15.*4.0;
t65 = t54+t55+t64-t69-t70-t71;
t72 = AddedMmgz.*t49.*t65;
t74 = t3.*t5.*t9.*t15.*t19.*4.0;
t73 = t53-t58+t59+t60-t61-t74;
t75 = t18.*t28.*4.0;
t76 = t9.*t15.*t25.*4.0;
t86 = t4.*t9.*t19.*t20.*4.0;
t77 = t75+t76-t86;
t78 = t17.*t18.*4.0;
t79 = t9.*t14.*t15.*4.0;
t80 = t5.*t9.*t19.*t20.*4.0;
t81 = t78+t79+t80;
t82 = t39-t52;
t83 = t42+t43;
t84 = t46+t48;
t85 = -t53+t58+t61;
t87 = AddedMmgy.*t44.*t81;
t88 = t87-AddedMmgx.*t40.*t77;
t89 = AddedMmgx.*t73.*t77;
t90 = t89-AddedMmgy.*t63.*t81;
t91 = t56+t57-t66;
t92 = -AddedMmgx.*t40.*t82-AddedMmgy.*t44.*t83-AddedMmgz.*t49.*t84;
t93 = AddedMmgx.*t73.*t82;
t94 = AddedMmgy.*t63.*t83;
t95 = t93+t94-AddedMmgz.*t65.*t84;
t96 = AddedMmgx.*t77.*t82;
t97 = t96-AddedMmgy.*t81.*t83;
t98 = t54+t55-t70;
t99 = AddedMmgx.*t73.*t85;
t100 = t99-AddedMmgy.*t63.*t91-AddedMmgz.*t65.*t98;
t101 = AddedMmgx.*t77.*t85;
t102 = AddedMmgy.*t81.*t91;
t103 = t101+t102;
t104 = AddedMmgx.*t82.*t85;
t105 = AddedMmgz.*t84.*t98;
t106 = t104+t105-AddedMmgy.*t83.*t91;
AddedMassGlider = reshape([AddedMmgx,0.0,0.0,-AddedMmgx.*t40,AddedMmgx.*t73,AddedMmgx.*t77,AddedMmgx.*t82,AddedMmgx.*t85,0.0,0.0,0.0,0.0,0.0,AddedMmgy,0.0,AddedMmgy.*t44,-AddedMmgy.*t63,AddedMmgy.*t81,-AddedMmgy.*t83,AddedMmgy.*t91,0.0,0.0,0.0,0.0,0.0,0.0,AddedMmgz,AddedMmgz.*t49,AddedMmgz.*t65,0.0,-AddedMmgz.*t84,-AddedMmgz.*t98,0.0,0.0,0.0,0.0,AddedMmgx.*(t21+t14.*t18.*8.0-t9.*t15.*t17.*8.0-t5.*t9.*t15.*t19.*8.0).*(-1.0./2.0),AddedMmgy.*(t30+t31+t18.*t25.*8.0-t9.*t15.*t28.*8.0).*(1.0./2.0),AddedMmgz.*(t34+t35+t3.*t18.*t19.*8.0+t6.*t9.*t15.*t19.*8.0).*(1.0./2.0),AddedMmgx.*t40.^2+AddedMmgy.*t44.^2+AddedMmgz.*t49.^2,t72-AddedMmgy.*t44.*t63-AddedMmgx.*t40.*t73,t88,t92,-AddedMmgx.*t40.*t85+AddedMmgy.*t44.*t91-AddedMmgz.*t49.*t98,0.0,0.0,0.0,0.0,AddedMmgx.*(t23-t14.*t15.*t18.*8.0+t5.*t8.*t9.*t20.*8.0+t5.*t6.*t18.*t19.*8.0-t5.*t18.*t19.*t20.*8.0-t3.*t5.*t9.*t15.*t19.*8.0).*(1.0./2.0),AddedMmgy.*(t32+t33-t15.*t18.*t25.*8.0-t4.*t8.*t9.*t20.*8.0-t4.*t6.*t18.*t19.*8.0+t3.*t4.*t9.*t15.*t19.*8.0).*(-1.0./2.0),AddedMmgz.*(t36+t37+t6.*t8.*t18.*8.0-t8.*t18.*t20.*8.0-t9.*t19.*t20.*8.0-t3.*t8.*t9.*t15.*8.0).*(1.0./2.0),t72-AddedMmgx.*t40.*(t53+t59+t60-t14.*t15.*t18.*4.0-t5.*t18.*t19.*t20.*4.0-t3.*t5.*t9.*t15.*t19.*4.0)-AddedMmgy.*t44.*t63,AddedMmgy.*t63.^2+AddedMmgz.*t65.^2+AddedMmgx.*t73.^2,t90,t95,t100,0.0,0.0,0.0,0.0,AddedMmgx.*(t18.*t28.*8.0+t9.*t15.*t25.*8.0-t4.*t9.*t19.*t20.*8.0).*(1.0./2.0),AddedMmgy.*(t17.*t18.*8.0+t9.*t14.*t15.*8.0+t5.*t9.*t19.*t20.*8.0).*(1.0./2.0),0.0,t88,t90,AddedMmgx.*t77.^2+AddedMmgy.*t81.^2,t97,t103,0.0,0.0,0.0,0.0,AddedMmgx.*(t21-t5.*t9.*t15.*t19.*8.0).*(1.0./2.0),AddedMmgy.*(t30+t31).*(-1.0./2.0),AddedMmgz.*(t34+t35).*(-1.0./2.0),t92,t95,t97,AddedMmgx.*t82.^2+AddedMmgy.*t83.^2+AddedMmgz.*t84.^2,t106,0.0,0.0,0.0,0.0,AddedMmgx.*(-t23+t14.*t15.*t18.*8.0+t5.*t18.*t19.*t20.*8.0).*(1.0./2.0),AddedMmgy.*(t32+t33-t15.*t18.*t25.*8.0).*(1.0./2.0),AddedMmgz.*(t36+t37-t8.*t18.*t20.*8.0).*(-1.0./2.0),-AddedMmgz.*t49.*(t54+t55-t8.*t18.*t20.*4.0)+AddedMmgy.*t44.*(t56+t57-t15.*t18.*t25.*4.0)-AddedMmgx.*t40.*t85,t100,t103,t106,AddedMmgx.*t85.^2+AddedMmgy.*t91.^2+AddedMmgz.*t98.^2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,AddedMJgx,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,AddedMJgy,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,AddedMJgz,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[12, 12]);
