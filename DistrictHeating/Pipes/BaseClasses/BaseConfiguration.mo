within DistrictHeating.Pipes.BaseClasses;
partial model BaseConfiguration "Base for DH configuration models"

  //Shape factors
  parameter Real hs;
  parameter Real ha;

  extends DistrictHeatingPipeParameters;

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
