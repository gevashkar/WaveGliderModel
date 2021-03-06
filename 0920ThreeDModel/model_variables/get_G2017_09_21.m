function G = get_G2017_09_21(in1,in2,ks)
%GET_G2017_09_21
%    G = GET_G2017_09_21(IN1,IN2,KS)

%    This function was generated by the Symbolic Math Toolbox version 5.10.
%    21-Sep-2017 16:48:31

phif = in1(4,:);
phig = in1(9,:);
phit = in1(7,:);
psif = in1(6,:);
psig = in1(11,:);
thetaa = in1(12,:);
thetaf = in1(5,:);
thetag = in1(10,:);
thetat = in1(8,:);
t2 = phif-phit;
t3 = cos(t2);
t4 = phif-phig;
t5 = cos(t4);
t6 = thetaf-thetat;
t7 = cos(t6);
t8 = sin(t4);
t9 = sin(t2);
t10 = pi.*(1.0./2.0);
t11 = t10+thetag-thetat;
t12 = thetaa-thetag;
t13 = cos(t11);
t14 = t3.*t5;
t15 = t3.*t5.*t7;
t25 = t8.*t9;
t37 = t7.*t8.*t9;
t16 = t14+t15-t25-t37;
t17 = sin(t6);
t18 = sin(t11);
t19 = psif-psig;
t20 = -t10+thetaf;
t21 = cos(t20);
t22 = sin(t20);
t23 = cos(t12);
t24 = cos(t19);
t26 = t3.*t17.*t18;
t27 = sin(t19);
t28 = t5.*t9;
t29 = t3.*t7.*t8;
t30 = t3.*t7.*4.0;
t31 = sin(t12);
t32 = t15-t25;
t33 = cos(phif);
t34 = t3.*t8;
t35 = t5.*t7.*t9;
t36 = t28+t29+t34+t35;
t38 = sin(phif);
t39 = t13.*t32;
t40 = t26+t39;
t41 = t24.*t40;
t42 = t28+t29;
t43 = t27.*t42;
t44 = t41+t43;
t45 = t18.*t32;
t46 = sin(psif);
t47 = cos(psif);
t48 = t17.*4.0;
t49 = t7.*t18;
t56 = t5.*t13.*t17;
t50 = t49-t56;
t51 = t33.*t46;
t146 = t21.*t38.*t47;
t52 = t51-t146;
t53 = t7.*t13;
t54 = t5.*t17.*t18;
t55 = t53+t54;
t57 = t24.*t50;
t80 = t8.*t17.*t27;
t58 = t57-t80;
t59 = t38.*t46;
t60 = t21.*t33.*t47;
t61 = t59+t60;
t62 = t7.*t9.*4.0;
t63 = t9.*t17.*t18;
t64 = t34+t35;
t65 = t13.*t64;
t66 = t63+t65;
t67 = t14-t37;
t69 = t23.*t44.*(2.2e1./1.25e2);
t70 = t3.*t13.*t17;
t288 = t45-t70;
t71 = t31.*t288.*(2.2e1./1.25e2);
t72 = t30-t69+t71;
t73 = t24.*t50.*(2.13e2./1.0e3);
t74 = t8.*t17.*t27.*(2.13e2./1.0e3);
t75 = t48+t73-t74;
t79 = t31.*t55.*(2.2e1./1.25e2);
t81 = t23.*t58.*(2.2e1./1.25e2);
t82 = t48+t79+t81;
t85 = t24.*t40.*(2.13e2./1.0e3);
t86 = t27.*t42.*(2.13e2./1.0e3);
t87 = t24.*t66.*(2.13e2./1.0e3);
t88 = t27.*t67.*(2.13e2./1.0e3);
t89 = t62-t87+t88;
t90 = t24.*t66;
t91 = t27.*t67;
t92 = t90-t91;
t93 = t23.*t92.*(2.2e1./1.25e2);
t94 = t18.*t64;
t95 = t9.*t13.*t17;
t96 = t94-t95;
t97 = t31.*t96.*(2.2e1./1.25e2);
t98 = t62-t93+t97;
t114 = -t30+t85+t86;
t145 = t61.*t72;
t147 = t52.*t75;
t148 = t52.*t82;
t149 = t22.*t47.*t89;
t150 = t22.*t47.*t98;
t155 = t61.*t114;
t68 = t145-t147+t148-t149+t150+t155;
t76 = t33.*t47;
t77 = t21.*t38.*t46;
t78 = t76+t77;
t83 = t38.*t47;
t101 = t21.*t33.*t46;
t84 = t83-t101;
t102 = t72.*t84;
t103 = t75.*t78;
t104 = t78.*t82;
t105 = t84.*(-t30+t85+t86);
t106 = t22.*t46.*t89;
t107 = t22.*t46.*t98;
t99 = t102-t103+t104+t105+t106-t107;
t122 = t21.*t89;
t123 = t21.*t98;
t124 = t22.*t33.*(-t30+t85+t86);
t125 = t22.*t33.*t72;
t126 = t22.*t38.*t75;
t127 = t22.*t38.*t82;
t100 = t122-t123+t124+t125+t126-t127;
t108 = t5.*t17.*t27;
t141 = t8.*t13.*t17.*t24;
t109 = t108-t141;
t110 = t13.*t36;
t111 = t63+t110;
t112 = t16.*t27;
t113 = t18.*t36;
t115 = t13.*t16;
t116 = t26+t115;
t117 = t16.*t18;
t118 = t24.*t116;
t119 = t27.*t36;
t120 = t118+t119;
t139 = t24.*t111;
t121 = t112-t139;
t128 = t24.*t116.*(2.13e2./1.0e3);
t129 = t27.*t36.*(2.13e2./1.0e3);
t130 = -t30+t128+t129;
t131 = t70-t117;
t132 = t31.*t131.*(2.2e1./1.25e2);
t133 = t23.*t120.*(2.2e1./1.25e2);
t134 = -t30+t132+t133;
t135 = t16.*t27.*(2.13e2./1.0e3);
t154 = t24.*t111.*(2.13e2./1.0e3);
t136 = t62+t135-t154;
t137 = t5.*t17.*t27.*(2.13e2./1.0e3);
t151 = t8.*t13.*t17.*t24.*(2.13e2./1.0e3);
t138 = t137-t151;
t140 = t95-t113;
t142 = t23.*t109.*(2.2e1./1.25e2);
t143 = t8.*t17.*t18.*t31.*(2.2e1./1.25e2);
t144 = t142+t143;
t152 = t23.*(t112-t139).*(2.2e1./1.25e2);
t153 = t62+t152-t31.*t140.*(2.2e1./1.25e2);
t162 = t61.*(-t30+t85+t86);
t156 = t145-t147+t148-t149+t150+t162;
t157 = t99.^2;
t158 = t100.^2;
t159 = t31.*t55.*(1.1e1./1.25e2);
t160 = t23.*t58.*(1.1e1./1.25e2);
t161 = t48+t159+t160;
t163 = t156.^2;
t164 = t157+t158+t163;
t165 = t7.*t9.*t18;
t171 = t5.*t9.*t13.*t17;
t166 = t165-t171;
t167 = t9.*t17.*4.0;
t168 = t7.*t9.*t13;
t169 = t5.*t9.*t17.*t18;
t170 = t168+t169;
t172 = t24.*t166;
t207 = t8.*t9.*t17.*t27;
t173 = t172-t207;
t174 = t17.*t18;
t175 = t5.*t7.*t13;
t176 = t174+t175;
t177 = t13.*t17;
t195 = t5.*t7.*t18;
t178 = t177-t195;
t179 = t24.*t176;
t180 = t7.*t8.*t27;
t181 = t179+t180;
t182 = t3.*t7.*t18;
t188 = t3.*t5.*t13.*t17;
t183 = t182-t188;
t184 = t3.*t17.*4.0;
t185 = t3.*t7.*t13;
t186 = t3.*t5.*t17.*t18;
t187 = t185+t186;
t189 = t24.*t183;
t201 = t3.*t8.*t17.*t27;
t190 = t189-t201;
t191 = t24.*t176.*(2.13e2./1.0e3);
t192 = t7.*t8.*t27.*(2.13e2./1.0e3);
t194 = t7.*4.0;
t193 = t191+t192-t194;
t196 = t31.*t178.*(2.2e1./1.25e2);
t197 = t23.*t181.*(2.2e1./1.25e2);
t198 = t24.*t183.*(2.13e2./1.0e3);
t211 = t3.*t8.*t17.*t27.*(2.13e2./1.0e3);
t199 = t184+t198-t211;
t200 = t31.*t187.*(2.2e1./1.25e2);
t202 = t23.*t190.*(2.2e1./1.25e2);
t203 = t184+t200+t202;
t204 = t24.*t166.*(2.13e2./1.0e3);
t212 = t8.*t9.*t17.*t27.*(2.13e2./1.0e3);
t205 = t167+t204-t212;
t206 = t31.*t170.*(2.2e1./1.25e2);
t208 = t23.*t173.*(2.2e1./1.25e2);
t209 = t167+t206+t208;
t210 = -t194+t196+t197;
t213 = 1.0./sqrt(t164);
t214 = sqrt(t164);
t215 = t214-3.7e1./1.0e3;
t216 = t27.*t66;
t217 = t24.*t67;
t218 = t216+t217;
t219 = t27.*t50.*(2.13e2./1.0e3);
t220 = t8.*t17.*t24.*(2.13e2./1.0e3);
t221 = t219+t220;
t222 = t24.*t42.*(2.13e2./1.0e3);
t232 = t27.*t40.*(2.13e2./1.0e3);
t223 = t222-t232;
t224 = t27.*t50;
t225 = t8.*t17.*t24;
t226 = t224+t225;
t227 = t24.*t42;
t233 = t27.*t40;
t228 = t227-t233;
t229 = t27.*t66.*(2.13e2./1.0e3);
t230 = t24.*t67.*(2.13e2./1.0e3);
t231 = t229+t230;
t234 = t31.*(t45-t70).*(1.1e1./1.25e2);
t235 = t30+t234-t23.*t44.*(1.1e1./1.25e2);
t236 = t31.*t96.*(1.1e1./1.25e2);
t237 = t62+t236-t23.*t92.*(1.1e1./1.25e2);
t238 = t9.*t17.*t21.*6.0822e3;
t239 = t94-t95-t165+t171;
t240 = t63+t65-t168-t169;
t241 = t24.*t239;
t242 = t207+t241;
t243 = t53+t54-t174-t175;
t244 = t26+t39-t185-t186;
t245 = t45-t70-t182+t188;
t246 = t24.*t245;
t247 = t201+t246;
t248 = t49-t56+t177-t195;
t255 = t24.*t243;
t249 = t180-t255;
t250 = t31.*t244.*(2.2e1./1.25e2);
t251 = t23.*t247.*(2.2e1./1.25e2);
t252 = -t184+t250+t251;
t253 = t24.*t245.*(2.13e2./1.0e3);
t254 = t31.*t248.*(2.2e1./1.25e2);
t256 = t23.*t249.*(2.2e1./1.25e2);
t257 = t24.*t243.*(2.13e2./1.0e3);
t258 = -t192+t194+t257;
t259 = t24.*t239.*(2.13e2./1.0e3);
t260 = -t167+t212+t259;
t261 = t31.*t240.*(2.2e1./1.25e2);
t262 = t23.*t242.*(2.2e1./1.25e2);
t263 = -t167+t261+t262;
t264 = t23.*t109.*(1.1e1./1.25e2);
t265 = t8.*t17.*t18.*t31.*(1.1e1./1.25e2);
t266 = t264+t265;
t267 = t22.*t38.*t266.*(9.81e2./5.0e1);
t268 = t52.*t138;
t269 = t27.*t32;
t276 = t13.*t24.*t42;
t270 = t269-t276;
t271 = t27.*t64;
t272 = t13.*t24.*t67;
t273 = t271+t272;
t274 = t78.*t138;
t275 = t27.*t32.*(2.13e2./1.0e3);
t277 = t23.*t270.*(2.2e1./1.25e2);
t278 = t18.*t31.*t42.*(2.2e1./1.25e2);
t279 = t277+t278;
t280 = t23.*t273.*(2.2e1./1.25e2);
t285 = t18.*t31.*t67.*(2.2e1./1.25e2);
t281 = t280-t285;
t282 = t27.*t64.*(2.13e2./1.0e3);
t283 = t13.*t24.*t67.*(2.13e2./1.0e3);
t284 = t282+t283;
t286 = t22.*t38.*t138;
t287 = t275-t13.*t24.*t42.*(2.13e2./1.0e3);
t290 = t31.*(t45-t70).*(2.2e1./1.25e2);
t291 = t30-t69+t290;
t294 = t61.*t291;
t289 = -t147+t148-t149+t150+t162+t294;
t301 = t84.*t291;
t292 = -t103+t104+t105+t106-t107+t301;
t305 = t22.*t33.*t291;
t293 = t122-t123+t124+t126-t127+t305;
t295 = t23.*t55.*(2.2e1./1.25e2);
t296 = t31.*t50.*(2.2e1./1.25e2);
t310 = t31.*t58.*(2.2e1./1.25e2);
t311 = t23.*t24.*t55.*(2.2e1./1.25e2);
t297 = t295+t296-t310-t311;
t298 = t31.*t66.*(2.2e1./1.25e2);
t299 = t23.*t24.*t96.*(2.2e1./1.25e2);
t306 = t31.*t92.*(2.2e1./1.25e2);
t307 = t23.*t96.*(2.2e1./1.25e2);
t300 = t298+t299-t306-t307;
t302 = t31.*t40.*(2.2e1./1.25e2);
t303 = t23.*t24.*t288.*(2.2e1./1.25e2);
t308 = t31.*t44.*(2.2e1./1.25e2);
t309 = t23.*t288.*(2.2e1./1.25e2);
t304 = t302+t303-t308-t309;
t312 = t289.^2;
t313 = t292.^2;
t314 = t293.^2;
t315 = t312+t313+t314;
t316 = t22.*t23.*t38.*t226.*1.72656;
t317 = sqrt(t315);
t318 = t317-3.7e1./1.0e3;
t319 = 1.0./sqrt(t315);
t320 = t52.*t221;
t321 = t61.*t223;
t322 = t22.*t23.*t47.*t218.*(2.2e1./1.25e2);
t323 = t23.*t78.*t226.*(2.2e1./1.25e2);
t324 = t23.*t84.*t228.*(2.2e1./1.25e2);
t325 = t22.*t23.*t46.*t218.*(2.2e1./1.25e2);
t326 = t21.*t231;
t327 = t22.*t33.*t223;
t328 = t22.*t23.*t38.*t226.*(2.2e1./1.25e2);
t329 = t326+t327+t328-t21.*t23.*t218.*(2.2e1./1.25e2)-t22.*t38.*t221-t22.*t23.*t33.*t228.*(2.2e1./1.25e2);
t330 = t23.*t55.*(1.1e1./1.25e2);
t331 = t295-t310;
t332 = t306+t307;
t333 = t308+t309;
G = [0.0;0.0;-2.07972e3;t267-t21.*(t30-t23.*t120.*(1.1e1./1.25e2)+t31.*(t117-t3.*t13.*t17).*(1.1e1./1.25e2)).*(9.81e2./5.0e1)-t3.*t7.*t21.*6.0822e3-t17.*t22.*t33.*6.0822e3-t22.*t33.*t161.*(9.81e2./5.0e1)-t22.*t38.*(t30-t23.*t44.*(1.1e1./1.25e2)+t31.*(t45-t3.*t13.*t17).*(1.1e1./1.25e2)).*(9.81e2./5.0e1)-t22.*t33.*(t62+t23.*t121.*(1.1e1./1.25e2)+t31.*(t113-t9.*t13.*t17).*(1.1e1./1.25e2)).*(9.81e2./5.0e1)-t3.*t7.*t22.*t38.*6.0822e3-t7.*t9.*t22.*t33.*6.0822e3+ks.*t213.*(sqrt(t157+t158+t68.^2)-3.7e1./1.0e3).*(t99.*(t274-t84.*(t62+t23.*t121.*(2.2e1./1.25e2)-t31.*t140.*(2.2e1./1.25e2))+t72.*t78+t75.*t84-t82.*t84+t78.*t114+t84.*t136-t78.*t144-t22.*t46.*t130+t22.*t46.*t134).*2.0+t68.*(t268+t52.*t72+t61.*t75-t61.*t82+t52.*t114-t52.*t144+t61.*t136-t61.*t153+t22.*t47.*t130-t22.*t47.*t134).*2.0-t100.*(t286+t21.*t130-t21.*t134+t22.*t38.*(-t30+t85+t86)-t22.*t33.*t75+t22.*t38.*t72+t22.*t33.*t82-t22.*t33.*t136-t22.*t38.*t144+t22.*t33.*t153).*2.0).*(1.0./2.0);t238+t21.*(t167+t23.*t173.*(1.1e1./1.25e2)+t31.*t170.*(1.1e1./1.25e2)).*(9.81e2./5.0e1)+t22.*t237.*(9.81e2./5.0e1)+t22.*t38.*(t7.*-4.0+t23.*t181.*(1.1e1./1.25e2)+t31.*t178.*(1.1e1./1.25e2)).*(9.81e2./5.0e1)-t22.*t33.*(t184+t23.*t190.*(1.1e1./1.25e2)+t31.*t187.*(1.1e1./1.25e2)).*(9.81e2./5.0e1)+t7.*t9.*t22.*6.0822e3-t7.*t22.*t38.*6.0822e3-t17.*t21.*t38.*6.0822e3-t21.*t38.*t161.*(9.81e2./5.0e1)+t21.*t33.*t235.*(9.81e2./5.0e1)+t3.*t7.*t21.*t33.*6.0822e3-t3.*t17.*t22.*t33.*6.0822e3+ks.*t213.*t215.*(t99.*(t78.*t193+t84.*t199-t84.*t203-t78.*t210+t21.*t46.*t89-t21.*t46.*t98-t22.*t46.*t205+t22.*t46.*t209+t22.*t33.*t46.*t72+t22.*t38.*t46.*t75-t22.*t38.*t46.*t82+t22.*t33.*t46.*(-t30+t85+t86)).*2.0+t100.*(-t22.*t89+t22.*t98-t21.*t205+t21.*t209+t22.*t38.*(t7.*-4.0+t196+t197)+t21.*t33.*t72+t21.*t38.*t75-t21.*t38.*t82+t21.*t33.*t114-t22.*t38.*t193+t22.*t33.*t199-t22.*t33.*t203).*2.0-t156.*(-t52.*t193-t61.*t199+t52.*t210+t61.*t203+t21.*t47.*t89-t21.*t47.*t98-t22.*t47.*t205+t22.*t47.*t209+t22.*t33.*t47.*t72+t22.*t38.*t47.*t75-t22.*t38.*t47.*t82+t22.*t33.*t47.*t114).*2.0).*(1.0./2.0);t316-t21.*t23.*t218.*1.72656-t22.*t23.*t33.*t228.*1.72656+ks.*t213.*t215.*(t156.*(t102-t103+t104+t105+t106-t107+t320+t321+t322-t22.*t47.*t231-t23.*t52.*t226.*(2.2e1./1.25e2)-t23.*t61.*t228.*(2.2e1./1.25e2)).*2.0+t100.*t329.*2.0-t99.*(t145-t147+t148-t149+t150+t155+t323+t324+t325-t78.*t221-t84.*t223-t22.*t46.*t231).*2.0).*(1.0./2.0);t21.*t235.*(9.81e2./5.0e1)+t3.*t7.*t21.*6.0822e3+t22.*t33.*t237.*(9.81e2./5.0e1)+t7.*t9.*t22.*t33.*6.0822e3+ks.*t213.*t215.*(t100.*(t21.*t72+t21.*t114-t22.*t33.*t89+t22.*t33.*t98).*2.0+t99.*(-t84.*t89+t84.*t98+t22.*t46.*t72+t22.*t46.*t114).*2.0-t156.*(t61.*t89-t61.*t98+t22.*t47.*(-t30+t85+t86)+t22.*t47.*t72).*2.0).*(1.0./2.0);-t238+t21.*(-t167+t23.*t242.*(1.1e1./1.25e2)+t31.*t240.*(1.1e1./1.25e2)).*(9.81e2./5.0e1)-t22.*t33.*(-t184+t23.*t247.*(1.1e1./1.25e2)+t31.*t244.*(1.1e1./1.25e2)).*(9.81e2./5.0e1)-t22.*t38.*(-t194+t23.*t249.*(1.1e1./1.25e2)+t31.*t248.*(1.1e1./1.25e2)).*(9.81e2./5.0e1)+t7.*t22.*t38.*6.0822e3+t3.*t17.*t22.*t33.*6.0822e3+ks.*t213.*t215.*(t156.*(t52.*t258-t61.*t252+t61.*(-t184+t211+t253)+t52.*(-t194+t254+t256)+t22.*t47.*t260-t22.*t47.*t263).*2.0+t99.*(t78.*t258-t84.*t252+t84.*(-t184+t211+t253)+t78.*(-t194+t254+t256)-t22.*t46.*t260+t22.*t46.*t263).*2.0-t100.*(t21.*t260-t21.*t263-t22.*t33.*(-t184+t211+t253)+t22.*t38.*(-t194+t254+t256)+t22.*t33.*t252+t22.*t38.*t258).*2.0).*(1.0./2.0);-t267-t21.*(t23.*t273.*(1.1e1./1.25e2)-t18.*t31.*t67.*(1.1e1./1.25e2)).*(9.81e2./5.0e1)+t22.*t33.*(t23.*t270.*(1.1e1./1.25e2)+t18.*t31.*t42.*(1.1e1./1.25e2)).*(9.81e2./5.0e1)-ks.*t213.*t215.*(t100.*(t286-t21.*t281+t21.*t284-t22.*t38.*t144+t22.*t33.*t279-t22.*t33.*t287).*-2.0+t156.*(t268-t52.*t144-t61.*t279+t61.*t287-t22.*t47.*t281+t22.*t47.*t284).*2.0+t99.*(t274-t78.*t144-t84.*t279+t84.*t287+t22.*t46.*t281-t22.*t46.*t284).*2.0).*(1.0./2.0);t21.*(t31.*t66.*(1.1e1./1.25e2)-t23.*t96.*(1.1e1./1.25e2)-t31.*t92.*(1.1e1./1.25e2)+t23.*t24.*t96.*(1.1e1./1.25e2)).*(-9.81e2./5.0e1)+t22.*t33.*(t31.*(t26+t39).*(1.1e1./1.25e2)-t31.*t44.*(1.1e1./1.25e2)-t23.*t288.*(1.1e1./1.25e2)+t23.*t24.*(t45-t70).*(1.1e1./1.25e2)).*(9.81e2./5.0e1)+t22.*t38.*(t330+t31.*t50.*(1.1e1./1.25e2)-t31.*t58.*(1.1e1./1.25e2)-t23.*t24.*t55.*(1.1e1./1.25e2)).*(9.81e2./5.0e1)-ks.*t318.*t319.*(t293.*(-t21.*t300+t21.*t24.*t96.*(2.13e2./1.0e3)+t22.*t38.*t297+t22.*t33.*t304+t22.*t24.*t38.*t55.*(2.13e2./1.0e3)-t22.*t24.*t33.*t288.*(2.13e2./1.0e3)).*-2.0+t289.*(t52.*t297-t61.*t304+t24.*t52.*t55.*(2.13e2./1.0e3)-t22.*t47.*t300+t24.*t61.*t288.*(2.13e2./1.0e3)+t22.*t24.*t47.*t96.*(2.13e2./1.0e3)).*2.0+t292.*(t78.*t297-t84.*t304+t24.*t55.*t78.*(2.13e2./1.0e3)+t22.*t46.*t300+t24.*t84.*t288.*(2.13e2./1.0e3)-t22.*t24.*t46.*t96.*(2.13e2./1.0e3)).*2.0).*(1.0./2.0);-t316+t21.*t23.*t218.*1.72656+t22.*t23.*t33.*t228.*1.72656-ks.*t318.*t319.*(t293.*t329.*2.0+t289.*(t320+t321+t322-t22.*t47.*t231-t23.*t52.*t226.*(2.2e1./1.25e2)-t23.*t61.*t228.*(2.2e1./1.25e2)).*2.0-t292.*(t323+t324+t325-t78.*t221-t84.*t223-t22.*t46.*t231).*2.0).*(1.0./2.0);t21.*(t23.*t96.*(1.1e1./1.25e2)+t31.*t92.*(1.1e1./1.25e2)).*(-9.81e2./5.0e1)+t22.*t33.*(t31.*t44.*(1.1e1./1.25e2)+t23.*t288.*(1.1e1./1.25e2)).*(9.81e2./5.0e1)-t22.*t38.*(t330-t31.*t58.*(1.1e1./1.25e2)).*(9.81e2./5.0e1)+ks.*t318.*t319.*(t289.*(t52.*t331+t61.*t333+t22.*t47.*t332).*2.0+t292.*(t78.*t331+t84.*t333-t22.*t46.*t332).*2.0-t293.*(t21.*t332-t22.*t33.*t333+t22.*t38.*t331).*2.0).*(1.0./2.0)];
