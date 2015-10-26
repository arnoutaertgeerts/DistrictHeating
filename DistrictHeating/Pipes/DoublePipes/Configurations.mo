within DistrictHeating.Pipes.DoublePipes;
package Configurations
  model TwinPipeGround "Twin pipes in the ground"

    //Extensions
    extends BaseClasses.BaseConfiguration(
      hs=1/hsInvers,
      ha=1/haInvers);

  protected
    final parameter Real hsInvers=
      2*lambdaI/lambdaG*Modelica.Math.log(2*Heff/rc) +
      Modelica.Math.log(rc^2/(2*e*ri)) +
      sigma*Modelica.Math.log(rc^4/(rc^4-e^4)) -
      (ri/(2*e) - sigma*2*ri*e^3/(rc^4-e^4))/(1+(ri/(2*e))^2 + sigma*(2*ri*rc^2*e/(rc^4-e^4))^2);
    final parameter Real haInvers=
      Modelica.Math.log(2*e/ri) +
      sigma*Modelica.Math.log((rc^2+e^2)/(rc^2-e^2)) -
      (ri/(2*e)-gamma*e*ri/(4*Heff^2)+2*sigma*ri*rc^2*e/(rc^4-e^4))^2/(1-(ri/(2*e))^2-gamma*ri/(2*e)+2*sigma*ri^2*rc^2*(rc^4+e^4)/((rc^4-e^4)^2))
      - gamma*(e/(2*Heff))^2;

    final parameter Real sigma = (lambdaI-lambdaG)/(lambdaI+lambdaG);
    final parameter Real gamma = 2*(1-sigma^2)/(1-sigma*(rc/(2*Heff))^2)
    annotation (Icon(graphics={
          Ellipse(
            extent={{-78,-30},{-18,30}},
            lineColor={0,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{18,-28},{78,32}},
            lineColor={0,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,255},
            fillColor={127,0,0},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-100,100},{100,-100}},
            lineColor={175,175,175},
            fillColor={255,255,255},
            fillPattern=FillPattern.Forward),
          Ellipse(
            extent={{-72,-28},{-12,32}},
            lineColor={0,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{10,-28},{70,32}},
            lineColor={0,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));

  end TwinPipeGround;

  model TwinPipeAir "Twin pipes above the ground"

    //Extensions
    extends BaseClasses.BaseConfiguration(
      hs=1/hsInvers,
      ha=1/haInvers);

  protected
    parameter Real hsInvers=
      Modelica.Math.log(rc^2/(2*e*ri)) -
      Modelica.Math.log(rc^4/(rc^4-e^4));
    parameter Real haInvers=
      Modelica.Math.log(2*e/ri) -
      Modelica.Math.log((rc^2+e^2)/(rc^2-e^2));

    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
              {100,100}}), graphics={
          Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,255},
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-100,100},{100,-100}},
            lineColor={175,175,175},
            fillColor={255,255,255},
            fillPattern=FillPattern.Forward),
          Ellipse(
            extent={{-72,-28},{-12,32}},
            lineColor={0,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{10,-28},{70,32}},
            lineColor={0,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end TwinPipeAir;

  model TwinPipeSeparate "Twin pipes in separate insulation"

    //Extensions
    extends BaseClasses.BaseConfiguration(
      hs=1/hsInvers,
      ha=1/haInvers);
    //Parameters
  protected
    parameter Real hsInvers=
      Modelica.Math.log(2*Heff/ro) + beta +
      Modelica.Math.log(sqrt(1 + (Heff/e)^2));
    parameter Real haInvers=
      Modelica.Math.log(2*Heff/ro) + beta -
      Modelica.Math.log(sqrt(1 + (Heff/e)^2));

    annotation (Icon(graphics={
          Ellipse(
            extent={{-80,-34},{-12,34}},
            lineColor={0,0,255},
            fillColor={175,175,175},
            fillPattern=FillPattern.Forward),
          Ellipse(
            extent={{-76,-30},{-16,30}},
            lineColor={0,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{16,-30},{76,30}},
            lineColor={0,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{12,-34},{80,34}},
            lineColor={0,0,255},
            fillColor={175,175,175},
            fillPattern=FillPattern.Forward),
          Ellipse(
            extent={{16,-30},{76,30}},
            lineColor={0,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end TwinPipeSeparate;
end Configurations;
