within DistrictHeating.Pipes.BaseClasses;
model Pipe
  "Model of a pipe with finite volume discretization along the flow path"
  extends Buildings.Fluid.Interfaces.LumpedVolumeDeclarations;
  extends Buildings.Fluid.Interfaces.PartialTwoPortInterface(
  showDesignFlowDirection = false,
  final show_T=true);
  extends Buildings.Fluid.Interfaces.TwoPortFlowResistanceParameters(
    final computeFlowResistance=(abs(dp_nominal) > Modelica.Constants.eps));

  parameter Integer nSeg(min=1) = 10 "Number of volume segments";
  parameter Modelica.SIunits.Length thicknessIns "Thickness of insulation";
  parameter Modelica.SIunits.ThermalConductivity lambdaIns
    "Heat conductivity of insulation";
  parameter Modelica.SIunits.Length diameter
    "Pipe diameter (without insulation)";

  parameter Modelica.SIunits.Length length "Length of the pipe";
  parameter Real ReC=4000
    "Reynolds number where transition to turbulent starts"
    annotation (Dialog(tab="Flow resistance"));

  parameter Boolean homotopyInitialization = true "= true, use homotopy method"
    annotation(Evaluate=true, Dialog(tab="Advanced"));

  Buildings.Fluid.FixedResistances.FixedResistanceDpM preDro(
    redeclare final package Medium = Medium,
    final from_dp=from_dp,
    use_dh=true,
    dh=diameter,
    final show_T=show_T,
    final m_flow_nominal=m_flow_nominal,
    final dp_nominal=dp_nominal,
    final allowFlowReversal=allowFlowReversal,
    final linearized=linearizeFlowResistance,
    final ReC=ReC,
    final homotopyInitialization=homotopyInitialization) "Flow resistance"
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  Buildings.Fluid.MixingVolumes.MixingVolume[nSeg] vol(
    redeclare each final package Medium = Medium,
    each energyDynamics=energyDynamics,
    each massDynamics=massDynamics,
    each final V=VPipe/nSeg,
    each nPorts=2,
    each final m_flow_nominal=m_flow_nominal,
    each prescribedHeatFlowRate=true,
    each p_start=p_start,
    each T_start=T_start,
    each X_start=X_start,
    each C_start=C_start,
    each C_nominal=C_nominal,
    each final m_flow_small=m_flow_small,
    each final allowFlowReversal=allowFlowReversal) "Volume for pipe fluid"
                                                  annotation (Placement(
        transformation(extent={{53,-20},{73,-40}})));

protected
  parameter Modelica.SIunits.Volume VPipe=Modelica.Constants.pi*(diameter/2.0)^
      2*length "Pipe volume";
  parameter Medium.ThermodynamicState state_default = Medium.setState_pTX(
      T=Medium.T_default,
      p=Medium.p_default,
      X=Medium.X_default[1:Medium.nXi]) "Default state";
  parameter Modelica.SIunits.Density rho_default = Medium.density(state_default);
  parameter Modelica.SIunits.DynamicViscosity mu_default = Medium.dynamicViscosity(state_default)
    "Dynamic viscosity at nominal condition";
equation
  connect(port_a, preDro.port_a) annotation (Line(
      points={{-100,5.55112e-016},{-72,5.55112e-016},{-72,1.16573e-015},{-58,
          1.16573e-015},{-58,0},{20,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(preDro.port_b, vol[1].ports[1]) annotation (Line(
      points={{40,0},{61,0},{61,-20}},
      color={0,127,255},
      smooth=Smooth.None));
  if nSeg > 1 then
    for i in 1:(nSeg - 1) loop
      connect(vol[i].ports[2], vol[i + 1].ports[1]);
    end for;
  end if;
  connect(vol[nSeg].ports[2], port_b) annotation (Line(
      points={{65,-20},{66,-20},{66,0},{100,0}},
      color={0,127,255},
      smooth=Smooth.None));
  annotation (
    Icon(graphics={
        Rectangle(
          extent={{-100,60},{100,-60}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={192,192,192}),
        Rectangle(
          extent={{-100,50},{100,-48}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={217,236,256}),
        Text(
          extent={{-42,12},{40,-12}},
          lineColor={0,0,0},
          textString="%nSeg")}),
    Documentation(info="<html>
<p>
Model of a pipe with flow resistance and optional heat storage.
This model can be used for modeling the heat exchange between the pipe and environment.
The model consists of a flow resistance
<a href=\"modelica://Buildings.Fluid.FixedResistances.FixedResistanceDpM\">
Buildings.Fluid.FixedResistances.FixedResistanceDpM</a>
and <code>nSeg</code> mixing volumes
<a href=\"modelica://Buildings.Fluid.MixingVolumes.MixingVolume\">
Buildings.Fluid.MixingVolumes.MixingVolume</a>.
</p>
</html>", revisions="<html>
<ul>
<li>
February 5, 2015, by Michael Wetter:<br/>
Renamed <code>res</code> to <code>preDro</code> for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/292\">#292</a>.
</li>
<li>
October 10, 2014, by Michael Wetter:<br/>
Changed minimum attribute for <code>nSeg</code> from 2 to 1.
This is required for the radiant slab model.
</li>
<li>
October 8, 2013, by Michael Wetter:<br/>
Removed parameter <code>show_V_flow</code>.
</li>
<li>
September 13, 2013 by Michael Wetter:<br/>
Replaced <code>nominal</code> with <code>default</code> values
as they are computed using the default Medium values.
</li>
<li>
February 15, 2012 by Michael Wetter:<br/>
Changed base class from which the model extends.
Propagated parameters of volume to the top if this model.
</li>
<li>
February 12, 2012 by Wangda Zuo:<br/>
First implementation.
</li>
</ul>
</html>"));
end Pipe;