# Assembling the Battery

## Disassemble Packs
Make sure all cells are in approximately the same state of charge, when you assemble the packs.
The cell protection PCB does some balancing (20...50mA), but the cells should be in a similar state of charge when the packs are assembled.

To retrieve the cells, carefully disassemble the Tattu battery packs.
The cells are unprotected so make sure you don't short-circuit the contacts while soldering.
The bare cells should look like this:

<table align="center">
  <tr>
    <td align="center">
      <img src="images/how-to-assemble-battery/cell.jpg" alt="" width="300"/><br/>
    </td>
  </tr>
</table>

## Prepare Cell Adapter and Cell Protection PCBs
The [cell-adapter PCBs](electronics/gerbers/cell-adapter) should have two press-fit inserts for the zero mating height balancing connectors.
These connectors serve as contacts for balancing and for mechanical alignment during assembly.
Make sure the press fit inserts are in place and hold firmly in the PCB.
If the press fit is too loose and the inserts fall out, it will be very difficult to replace them later.
If you find the inserts too loose, apply a tiny bit of solder to the press fit hole to make the press fit tighter.
You can test with the cell protection PCBs (see below) if the alignment pins and press fit inserts fit.

<table align="center">
  <tr>
    <td align="center">
      <img src="images/how-to-assemble-battery/celladapter.png" alt="" width="300"/><br/>
    </td>
    <td align="center">
      <img src="images/how-to-assemble-battery/celladaptermarked.png" alt="" width="300"/><br/>
    </td>
  </tr>
</table>

If not already done, solder the thermistors to the [cell-protection PCBs](electronics/gerbers/cell-protection).
Make sure the alignment pins mate with the cell-adapter PCBs.
<table align="center">
  <tr>
    <td align="center">
      <img src="images/how-to-assemble-battery/protection0.jpg" alt="" width="300"/><br/>
    </td>
    <td align="center">
      <img src="images/how-to-assemble-battery/protection1.jpg" alt="" width="300"/><br/>
    </td>
    <td align="center">
      <img src="images/how-to-assemble-battery/protection3.jpg" alt="" width="300"/><br/>
    </td>
    <td align="center">
      <img src="images/how-to-assemble-battery/protection3marked.png" alt="" width="220"/><br/>
    </td>
  </tr>
</table>


## Solder Cells to Adapter
Solder three cells to the [cell-adapter PCBs](electronics/gerbers/cell-adapter).
Make sure to put the cells on the correct side (marked on the silk screen of the PCB with "cell on this side") and pay attention to the polarity as marked on the silk screen.
The cells should be perpendicular to the cell adapter PCB, avoid too much skew as this will put tension on the cell tabs when the final pack is inside the robot.


<table align="center">
  <tr>
    <td align="center">
      <img src="images/how-to-assemble-battery/threecells1.jpg" alt="" width="550"/><br/>
    </td>
    <td align="center">
      <img src="images/how-to-assemble-battery/threecells0.jpg" alt="" width="300"/><br/>
    </td>
    <td align="center">
      <img src="images/how-to-assemble-battery/threecells3.jpg" alt="" width="300"/><br/>
    </td>
    <td align="center">
      <img src="images/how-to-assemble-battery/threecells4.jpg" alt="" width="300"/><br/>
    </td>
    <td align="center">
      <img src="images/how-to-assemble-battery/threecells2.jpg" alt="" width="300"/><br/>
    </td>
  </tr>
</table>

You may want to cover the bare cell contacts with tape now, e.g., 3M ET 1339 tape.
<table align="center">
  <tr>
    <td align="center">
      <img src="images/how-to-assemble-battery/threecellscover.jpg" alt="" width="300"/><br/>
    </td>
  </tr>
</table>


## Assemble the Pack
Have 4x DIN912 M2x4mm socket head screw and 4x washer to hand.
Carefully align the cell adapter with its three cells to the cell protection PCB using the two pins and press-fit inserts.
When the cells are well aligned, carefully fasten with washer and screw without short-circuiting anywhere.
If your hands shake, use tape to temporarily mask contacts.
Repeat on the other side.
Tighten both M2 screws per cell adapter firmly as they make the contact that carries the main battery current.

<table align="center">
  <tr>
    <td align="center">
      <img src="images/how-to-assemble-battery/assemblybolted.jpg" alt="" width="300"/><br/>
    </td>
    <td align="center">
      <img src="images/how-to-assemble-battery/assemblybolted2.jpg" alt="" width="300"/><br/>
    </td>
  </tr>
</table>

Once bolted down, use 3M ET 1339 tape if you didn't previously, to cover the cell taps and wrap around the three pack of cells right below the cell adapter.
Then, use electrical grade silicone, e.g., Chip Quik EGS10C­20G, to generously back-fill any cavities between cells (marked B below) and cell-adapter and between cell adapter and soldered taps (marked A below).
The purpose of the silicone is to prevent the cell taps from crushing or buckling under mechanical pressure.
<table align="center">
  <tr>
    <td align="center">
      <img src="images/how-to-assemble-battery/siliconeA.png" alt="" width="280"/><br/>
    </td>
    <td align="center">
      <img src="images/how-to-assemble-battery/siliconeB.png" alt="" width="300"/><br/>
    </td>
  </tr>
</table>

Cut off pieces of shrink wrap to cover the cells.
These should be about 5-10mm longer than the cells. 
Use plastic tweezers or your fingers to bend the thermistors flat against the cells on each side and pull the shrink wrap over.
Then use a hot air gun to carefully shrink in the cells.
<table align="center">
  <tr>
    <td align="center">
      <img src="images/how-to-assemble-battery/assemblytape0.jpg" alt="" width="300"/><br/>
    </td>
    <td align="center">
      <img src="images/how-to-assemble-battery/assemblytape1.jpg" alt="" width="300"/><br/>
    </td>
  </tr>
</table>

Finally, cover in 3D printed housing and wrap with 3M ET 1339 tape to secure the lid on the housing.
<table align="center">
  <tr>
    <td align="center">
      <img src="images/how-to-assemble-battery/final.jpg" alt="" width="300"/><br/>
    </td>
    <td align="center">
      <img src="images/how-to-assemble-battery/final2.jpg" alt="" width="300"/><br/>
    </td>
  </tr>
</table>