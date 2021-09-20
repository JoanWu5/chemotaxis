package chemotaxis.g4;

import java.awt.Point;
import java.util.List;
import java.util.ArrayList;

import chemotaxis.sim.ChemicalPlacement;
import chemotaxis.sim.ChemicalCell;
import chemotaxis.sim.ChemicalCell.ChemicalType;
import chemotaxis.sim.SimPrinter;

public class Controller extends chemotaxis.sim.Controller {

	/**
	 * Controller constructor
	 *
	 * @param start       start cell coordinates
	 * @param target      target cell coordinates
	 * @param size     	  grid/map size
	 * @param simTime     simulation time
	 * @param budget      chemical budget
	 * @param seed        random seed
	 * @param simPrinter  simulation printer
	 *
	 */
	public Controller(Point start, Point target, Integer size, ChemicalCell[][] grid, Integer simTime, Integer budget, Integer seed, SimPrinter simPrinter) {
		super(start, target, size, grid, simTime, budget, seed, simPrinter);
	}

	public int closestToTarget(ArrayList<Point> locations) {
		int closestDistance = 9999999;
		int closestIdx = 0;
		for(int i = 0; i < locations.size(); i++) {
			int x = locations.get(i).x;
			int y = locations.get(i).y;
			int distance = Math.abs(x - this.target.x) + Math.abs(y - this.target.y);
			if(distance > 0 && distance < closestDistance) {
				closestIdx = i;
				closestDistance = distance;
			}
		}
		return closestIdx;
	}

	/**
	 * Apply chemicals to the map
	 *
	 * @param currentTurn         current turn in the simulation
	 * @param chemicalsRemaining  number of chemicals remaining
	 * @param locations     current locations of the agents
	 * @param grid                game grid/map
	 * @return                    a cell location and list of chemicals to apply
	 *
	 */
	@Override
	public ChemicalPlacement applyChemicals(Integer currentTurn, Integer chemicalsRemaining, ArrayList<Point> locations, ChemicalCell[][] grid) {

		Point target = this.target;

		int targetX = target.x;
		int targetY = target.y;

		ChemicalPlacement chemicalPlacement = new ChemicalPlacement();
		int closestIdx = this.closestToTarget(locations);
		Point currentLocation = locations.get(closestIdx);
		int currentX = currentLocation.x;
		int currentY = currentLocation.y;

		int diffx = targetX-currentX;
		int diffy = targetY-currentY;

		int blueX;
		int blueY;

		if (diffx > 0) {
			blueX = currentX + 1;
		}
		else {
			blueX = currentX - 1;
		}

		if (diffy > 0) {
			blueY = currentX + 1;
		}
		else {
			blueY = currentX - 1;
		}

		int leftEdgeX = Math.max(1, currentX - 5);
		int rightEdgeX = Math.min(size, currentX + 5);
		int topEdgeY = Math.max(1, currentY - 5);
		int bottomEdgeY = Math.min(size, currentY + 5);

		int randomX = this.random.nextInt(rightEdgeX - leftEdgeX + 1) + leftEdgeX;
		int randomY = this.random.nextInt(bottomEdgeY - topEdgeY + 1) + topEdgeY ;

		List<ChemicalType> chemicals = new ArrayList<>();
		chemicals.add(ChemicalType.BLUE);

		chemicalPlacement.location = new Point(blueX, blueY);
		chemicalPlacement.chemicals = chemicals;

		return chemicalPlacement;
	}
}
