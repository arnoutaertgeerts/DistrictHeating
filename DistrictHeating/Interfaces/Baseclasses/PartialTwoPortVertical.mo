within DistrictHeating.Interfaces.Baseclasses;
partial model PartialTwoPortVertical "Partial component with two ports"
  import Modelica.Constants;

  replaceable package Medium =Modelica.Media.Interfaces.PartialMedium
    "Medium in the component"
      annotation (choicesAllMatching = true);

  parameter Boolean allowFlowReversal = true
    "= true to allow flow reversal, false restricts to design direction (port_a -> port_b)"
    annotation(Dialog(tab="Assumptions"), Evaluate=true);

  Modelica.Fluid.Interfaces.FluidPort_a port_a(
    redeclare final package Medium = Medium,
     m_flow(min=if allowFlowReversal then -Constants.inf else 0))
    "Fluid connector a (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{-70,-110},{-50,-90}},
            rotation=0)));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(
    redeclare final package Medium = Medium,
    m_flow(max=if allowFlowReversal then +Constants.inf else 0))
    "Fluid connector b (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{70,-110},{50,-90}},rotation=
             0), iconTransformation(extent={{70,-110},{50,-90}})));
  // Model structure, e.g., used for visualization
protected
  parameter Boolean port_a_exposesState = false
    "= true if port_a exposes the state of a fluid volume";
  parameter Boolean port_b_exposesState = false
    "= true if port_b.p exposes the state of a fluid volume";
  parameter Boolean showDesignFlowDirection = true
    "= false to hide the arrow in the model icon";

  annotation (
    Documentation(info="<html>
<p>
This partial model defines an interface for components with two ports.
The treatment of the design flow direction and of flow reversal are predefined based on the parameter <code>allowFlowReversal</code>.
The component may transport fluid and may have internal storage for a given fluid <code>Medium</code>.
</p>
<p>
An extending model providing direct access to internal storage of mass or energy through <code>port_a</code> or <code>port_b</code>
should redefine the protected parameters <code>port_a_exposesState</code> and <code>port_b_exposesState</code> appropriately.
This will be visualized at the port icons, in order to improve the understanding of fluid model diagrams.
</p>
<h4>Implementation</h4>
<p>
This model is similar to
<a href=\"modelica://Modelica.Fluid.Interfaces.PartialTwoPort\">
Modelica.Fluid.Interfaces.PartialTwoPort</a>
but it does not use the <code>outer system</code> declaration.
This declaration is omitted as in building energy simulation,
many models use multiple media, an in practice,
users have not used this global definition to assign parameters.
</p>
</html>", revisions="<html>
<ul>
<li>
October 21, 2014, by Michael Wetter:<br/>
Revised implementation.
Declared medium in ports to be <code>final</code>.
</li>
<li>
October 20, 2014, by Filip Jorisson:<br/>
First implementation.
</li>
</ul>
</html>"),
    Icon(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={
        Text(
          extent={{-149,-114},{151,-154}},
          lineColor={0,0,255},
          textString="%name"),
        Ellipse(
          extent={{-110,26},{-90,-24}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          visible=port_a_exposesState),
        Ellipse(
          extent={{90,25},{110,-25}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          visible=port_b_exposesState)}),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
            100,100}}), graphics));
end PartialTwoPortVertical;
