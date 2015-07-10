within DistrictHeating.Pipes;
package Configurations
  model TwinPipeGround "Twin pipes in the ground"

    //Extensions
    extends BaseConfiguration(
      hs=1/hsInvers,
      ha=1/haInvers);

  protected
    final parameter Real hsInvers=
      2*lambdaI/lambdaG*Modelica.Math.log(2*Heff/rc) +
      Modelica.Math.log(rc^2/(2*e*ri)) +
      sigma*Modelica.Math.log(rc^4/(rc^4-e^4));
    final parameter Real haInvers=
      Modelica.Math.log(2*e/ri) +
      sigma*Modelica.Math.log((rc^2+e^2)/(rc^2-e^2));
    final parameter Real sigma = (lambdaI-lambdaG)/(lambdaI+lambdaG);

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
    extends BaseConfiguration(
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
    extends BaseConfiguration(
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

  partial model BaseConfiguration

    //Shape factors
    parameter Real hs;
    parameter Real ha;

    //Pipe dimensions
    parameter Modelica.SIunits.Length H=2 "Buried depth of the pipe";
    parameter Modelica.SIunits.Length E=1.25*Di
      "Horizontal distance between pipes";
    parameter Modelica.SIunits.Length Do=0.2 "Equivalent outer diameter";
    parameter Modelica.SIunits.Length Di=0.2 "Equivalent inner diameter";
    parameter Modelica.SIunits.Length Dc=2.5*Di
      "Diameter of circumscribing pipe";

    //Thermal conductivities
    parameter Modelica.SIunits.ThermalConductivity lambdaG=2
      "Thermal conductivity of the ground [W/mK]";
    parameter Modelica.SIunits.ThermalConductivity lambdaI=0.026
      "Thermal conductivity of the insulation [W/mK]";
    parameter Modelica.SIunits.ThermalConductivity lambdaGS = 14.6
      "Thermal conductivity of the ground surface [W/mK]";

    final parameter Real beta = lambdaG/lambdaI*Modelica.Math.log(ro/ri)
      "Dimensionless parameter describing the insulation";

    //Calculated parameters
  protected
    final parameter Modelica.SIunits.Length Heff=H + lambdaG/lambdaGS
      "Corrected depth";
    final parameter Modelica.SIunits.Length ro = Do/2 "Equivalent outer radius";
    final parameter Modelica.SIunits.Length ri = Di/2 "Equivalent inner radius";
    final parameter Modelica.SIunits.Length rc = Dc/2 "Circumscribing radius";
    final parameter Modelica.SIunits.Length e = E/2
      "Half the distance between the center of the pipes";

    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics), Icon(graphics={Ellipse(
            extent={{-78,-30},{-18,30}},
            lineColor={0,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid), Ellipse(
            extent={{18,-28},{78,32}},
            lineColor={0,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end BaseConfiguration;
end Configurations;
