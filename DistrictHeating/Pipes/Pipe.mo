within DistrictHeating.Pipes;
model Pipe
  "Pipe from Buildings, but adapted to use diameter instead of nominal flow as an input"
  extends Buildings.Fluid.FixedResistances.BaseClasses.Pipe(
   diameter=dh,
   dp_nominal=2*dpStraightPipe_nominal,
   preDro(dp(nominal=length*10)));
  // Because dp_nominal is a non-literal value, we set
  // dp.nominal=100 instead of the default dp.nominal=dp_nominal,
  // because the latter is ignored by Dymola 2012 FD 01.

  parameter Modelica.SIunits.Length dh = 0.05 "Hydraulic diameter of pipe";
  parameter Modelica.SIunits.Length roughness(min=0) = 2.5e-5
    "Absolute roughness of pipe, with a default for a smooth steel pipe (dummy if use_roughness = false)";
  final parameter Modelica.SIunits.Pressure dpStraightPipe_nominal=
      Modelica.Fluid.Pipes.BaseClasses.WallFriction.Detailed.pressureLoss_m_flow(
      m_flow=m_flow_nominal,
      rho_a=rho_default,
      rho_b=rho_default,
      mu_a=mu_default,
      mu_b=mu_default,
      length=length,
      diameter=dh,
      roughness=roughness,
      m_flow_small=m_flow_small)
    "Pressure loss of a straight pipe at m_flow_nominal";

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort
    "Single heat port that connects to outside of pipe wall (default, enabled when useMultipleHeatPorts=false)"
    annotation (Placement(transformation(extent={{-10,40},{10,20}}),
        iconTransformation(extent={{-10,60},{10,40}})));
equation

  connect(vol.heatPort, heatPort) annotation (Line(
      points={{-1,-28},{-46,-28},{-46,30},{0,30}},
      color={191,0,0},
      smooth=Smooth.None));
  annotation (
    defaultComponentName="pip",
    Documentation(info="<html>
<p>
Model of a pipe with flow resistance and optional heat exchange with environment.
</p>
<p>
If <code>useMultipleHeatPorts=false</code> (default option), the pipe uses a single heat port
for the heat exchange with the environment.
If <code>useMultipleHeatPorts=true</code>, then one heat port for each segment of the pipe is
used for the heat exchange with the environment.
If the heat port is unconnected, then the pipe has no heat loss.
</p>
<p>
The default value for the parameter <code>diameter</code> is computed such that the flow velocity
is equal to <code>v_nominal=0.15</code> for a mass flow rate of <code>m_flow_nominal</code>.
Both parameters, <code>diameter</code> and <code>v_nominal</code>, can be overwritten
by the user.
The default value for <code>dp_nominal</code> is two times the pressure drop that the pipe
would have if it were straight with no fittings.
The factor of two that takes into account the pressure loss of fittings can be overwritten.
These fittings could also be explicitly modeled outside of this component using models from
the package
<a href=\"modelica://Modelica.Fluid.Fittings\">
Modelica.Fluid.Fittings</a>.
For mass flow rates other than <code>m_flow_nominal</code>, the model
<a href=\"modelica://Buildings.Fluid.FixedResistances.FixedResistanceDpM\">
Buildings.Fluid.FixedResistances.FixedResistanceDpM</a> is used to
compute the pressure drop.
</p>
<p>
For a steady-state model of a flow resistance, use
<a href=\"modelica://Buildings.Fluid.FixedResistances.FixedResistanceDpM\">
Buildings.Fluid.FixedResistances.FixedResistanceDpM</a> instead of this model.
</p>
</html>", revisions="<html>
<ul>
<li>
February 5, 2015, by Michael Wetter:<br/>
Renamed <code>res</code> to <code>preDro</code> for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/292\">#292</a>.
</li>
<li>
September 13, 2013 by Michael Wetter:<br/>
Replaced <code>nominal</code> with <code>default</code> values
as they are computed using the default Medium values.
</li>
<li>
February 22, 2012 by Michael Wetter:<br/>
Renamed <code>useMultipleHeatPort</code> to <code>useMultipleHeatPorts</code> and
used heat port connector from <code>Modelica.Fluid</code> package for vector of heat ports.
</li>
<li>
February 15, 2012 by Michael Wetter:<br/>
Revised implementation and added default values.
</li>
<li>
February 12, 2012 by Wangda Zuo:<br/>
First implementation.
</li>
</ul>
</html>"),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
            100}}), graphics));
end Pipe;
