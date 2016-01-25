package org.lasarobotics.vision.ftc.resq;

import org.lasarobotics.vision.detection.PrimitiveDetection;
import org.lasarobotics.vision.detection.objects.Contour;
import org.lasarobotics.vision.detection.objects.Ellipse;
import org.lasarobotics.vision.detection.objects.Rectangle;
import org.lasarobotics.vision.image.Drawing;
import org.lasarobotics.vision.util.MathUtil;
import org.lasarobotics.vision.util.color.ColorGRAY;
import org.lasarobotics.vision.util.color.ColorRGBA;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;

import java.text.DecimalFormat;
import java.util.List;

/**
 * Beacon location and analysis
 */
public final class Beacon {

    public BeaconAnalysis analyzeColor(List<Contour> contoursRed, List<Contour> contoursBlue, Mat img, Mat gray, BeaconAnalysis currentState) {
        BeaconAnalysis result = new BeaconAnalysis(img.size());
        result.radius = currentState.getRadius();

        //The idea behind the SmartScoring algorithm is that the largest score in each contour/ellipse set will become the best
        //DONE First, ellipses and contours are are detected and pre-filtered to remove eccentricities
        //Second, ellipses, and contours are scored independently based on size and color ... higher score is better
        //TODO Third, comparative analysis is used on each ellipse and contour to create a score for the contours
        //Ellipses within rectangles strongly increase in value
        //Ellipses without nearby/contained contours are removed
        //Ellipses with nearby/contained contours associate themselves with the contour
        //Pairs of ellipses (those with similar size and x-position) greatly increase the associated contours' value
        //Contours without nearby/contained ellipses lose value
        //Contours near another contour of the opposite color increase in value
        //Contours and ellipses near the expected area (if any expected area) increase in value
        //Finally, a fraction of the ellipse value is added to the value of the contour
        //The best ellipse is found first, then only this ellipse adds to the value
        //TODO The best contour from each color (if available) is selected as red and blue
        //TODO The two best contours are then used to calculate the location of the beacon

        //TODO Filter out bad contours - filtering currently ignored

        //DEBUG Draw contours before filtering
        Drawing.drawContours(img, contoursRed, new ColorRGBA("#FFCDD2"), 2);
        Drawing.drawContours(img, contoursBlue, new ColorRGBA("#BBDEFB"), 2);

        //Score contours
        BeaconScoring scorer = new BeaconScoring(img.size());
        List<BeaconScoring.ScoredContour> scoredContoursRed = scorer.scoreContours(contoursRed, null, null, img, gray);
        List<BeaconScoring.ScoredContour> scoredContoursBlue = scorer.scoreContours(contoursBlue, null, null, img, gray);

        //DEBUG Draw red and blue contours after filtering
        Drawing.drawContours(img, BeaconScoring.ScoredContour.getList(scoredContoursRed), new ColorRGBA(255, 0, 0), 2);
        Drawing.drawContours(img, BeaconScoring.ScoredContour.getList(scoredContoursBlue), new ColorRGBA(0, 0, 255), 2);

        //Locate ellipses in the image to process contours against
        //Each contour must have an ellipse of correct specification
        PrimitiveDetection primitiveDetection = new PrimitiveDetection();
        PrimitiveDetection.EllipseLocationResult ellipseLocationResult = primitiveDetection.locateEllipses(gray);

        //Filter out bad ellipses - TODO filtering currently ignored
        List<Ellipse> ellipses = ellipseLocationResult.getEllipses();

        //DEBUG Ellipse data before filtering
        //Drawing.drawEllipses(img, ellipses, new ColorRGBA("#ff0745"), 1);

        //Score ellipses
        List<BeaconScoring.ScoredEllipse> scoredEllipses = scorer.scoreEllipses(ellipses, null, null, gray);

        //DEBUG Ellipse data after filtering
        Drawing.drawEllipses(img, BeaconScoring.ScoredEllipse.getList(scoredEllipses), new ColorRGBA("#FFC107"), 2);

        //DEBUG draw top 5 ellipses
        if (scoredEllipses.size() > 0) {
            Drawing.drawEllipses(img, BeaconScoring.ScoredEllipse.getList(scoredEllipses.subList(0, scoredEllipses.size() > 5 ? 5 : scoredEllipses.size()))
                    , new ColorRGBA("#d0ff00"), 3);
            Drawing.drawEllipses(img, BeaconScoring.ScoredEllipse.getList(scoredEllipses.subList(0, scoredEllipses.size() > 3 ? 3 : scoredEllipses.size()))
                    , new ColorRGBA("#00ff00"), 3);
        }

        //Third, comparative analysis is used on each ellipse and contour to create a score for the contours
        BeaconScoring.MultiAssociatedContours associations = scorer.scoreAssociations(scoredContoursRed, scoredContoursBlue, scoredEllipses);
        double score = (associations.blueContours.size() > 0 ? associations.blueContours.get(0).score : 0) +
                (associations.redContours.size() > 0 ? associations.redContours.get(0).score : 0);
        double confidence = score / Constants.CONFIDENCE_DIVISOR;

        //INFO The best contour from each color (if available) is selected as red and blue
        //INFO The two best contours are then used to calculate the location of the beacon

        //Get the best contour in each (starting with the largest) if any contours exist
        //We're calling this one the main light
        BeaconScoring.AssociatedContour bestAssociatedRed = (associations.redContours.size() > 0) ? associations.redContours.get(0) : null;
        BeaconScoring.AssociatedContour bestAssociatedBlue = (associations.blueContours.size() > 0) ? associations.blueContours.get(0) : null;

        Contour bestRed = (bestAssociatedRed != null) ? bestAssociatedRed.contour.contour : null;
        Contour bestBlue = (bestAssociatedBlue != null) ? bestAssociatedBlue.contour.contour : null;

        //Send movement data
        double centerSumX = (bestRed != null ? bestRed.center().x : 0) + (bestBlue != null ? bestBlue.center().x : 0);
        double centerSumY = (bestRed != null ? bestRed.center().y : 0) + (bestBlue != null ? bestBlue.center().y : 0);
        Point newWeightedCenter = new Point(centerSumX, centerSumY);
        result.beaconWeightedCenter = newWeightedCenter;

        //Check how much the weighted center of the beacon changed. If under threshold, set sameBeacon to true
        if(newWeightedCenter != null && (currentState != null) ? currentState.beaconWeightedCenter != null : false)
            result.sameBeacon = Math.abs(newWeightedCenter.x - currentState.beaconWeightedCenter.x) < Constants.MAX_CENTER_CHANGE*result.imageWidth
                    && Math.abs(newWeightedCenter.y - currentState.beaconWeightedCenter.y) < Constants.MAX_CENTER_CHANGE*result.imageHeight;
        else
            result.sameBeacon = false;

        //Send data for position calculations
        int numberOfEllipses = 0;
        boolean ellipseOne = bestAssociatedBlue != null ? bestAssociatedBlue.ellipses.size() > 0 : false;
        boolean ellipseTwo = bestAssociatedRed != null ? bestAssociatedRed.ellipses.size() > 0 : false;
        numberOfEllipses = (ellipseOne ? 1 : 0) + (ellipseTwo ? 1 : 0);
        double ellipseHeight = numberOfEllipses != 0 ? ((ellipseOne ? bestAssociatedBlue.ellipses.get(0).ellipse.height() : 0)
                + (ellipseTwo ? bestAssociatedRed.ellipses.get(0).ellipse.height() : 0))/numberOfEllipses : 0;
        double heightSum = (bestRed != null ? bestRed.height() : 0) + (bestBlue != null ? bestBlue.height() : 0);
        //TODO currently we assume that the robot is non stationary
        result.nonStationaryUpdate(heightSum, ellipseHeight, numberOfEllipses > 0);

        Drawing.drawText(img, "Distance: " + result.getRadius() * Constants.CM_FT_SCALE + " ft",
                new Point(0, 100), 1.0f, new ColorGRAY(255), Drawing.Anchor.BOTTOMLEFT);

        //If we don't have a main light for one of the colors, we know both colors are the same
        //TODO we should re-filter the contours by size to ensure that we get at least a decent size
        if (bestRed == null && bestBlue == null)
            return result;

        //TODO rotate image based on camera rotation here

        //The height of the beacon on screen is the height of the best contour
        Contour largestHeight = ((bestRed != null ? bestRed.size().height : 0) >
                (bestBlue != null ? bestBlue.size().height : 0)) ? bestRed : bestBlue;
        assert largestHeight != null;
        double beaconHeight = largestHeight.size().height;

        //Get beacon width on screen by extrapolating from height
        final double beaconActualHeight = Constants.BEACON_HEIGHT; //cm, only the lit up portion - 14.0 for entire
        final double beaconActualWidth = Constants.BEACON_WIDTH; //cm
        final double beaconWidthHeightRatio = Constants.BEACON_WH_RATIO;
        double beaconWidth = beaconHeight * beaconWidthHeightRatio;
        Size beaconSize = new Size(beaconWidth, beaconHeight);

        //If the largest part of the non-null color is wider than a certain distance, then both are bright
        //Otherwise, only one may be lit
        //If only one is lit, and is wider than a certain distance, it is bright
        //TODO We are currently assuming that the beacon cannot be in a "bright" state
        if (bestRed == null)
            return result;
        else if (bestBlue == null)
            return result;

        //Look at the locations of the largest contours
        //Check to see if the largest red contour is more left-most than the largest right contour
        //If it is, then we know that the left beacon is red and the other blue, and vice versa

        Point largestRedCenter = bestRed.center();
        Point largestBlueCenter = bestBlue.center();

        //DEBUG R/B text
        Drawing.drawText(img, "R", largestRedCenter, 1.0f, new ColorRGBA(255, 0, 0));
        Drawing.drawText(img, "B", largestBlueCenter, 1.0f, new ColorRGBA(0, 0, 255));

        //Test which side is red and blue
        //If the distance between the sides is smaller than a value, then return unknown
        final int xMinDistance = (int) (Constants.DETECTION_MIN_DISTANCE * beaconSize.width); //percent of beacon width
        boolean leftIsRed;
        if (largestRedCenter.x + xMinDistance < largestBlueCenter.x) {
            leftIsRed = true;
        } else if (largestBlueCenter.y + xMinDistance < largestRedCenter.x) {
            leftIsRed = false;
        } else {
            return result;
        }

        //Get the left-most best contour
        Contour leftMostContour = ((bestRed.topLeft().x) < (bestBlue.topLeft().x)) ? bestRed : bestBlue;
        //Get the right-most best contour
        Contour rightMostContour = ((bestRed.topLeft().x) < (bestBlue.topLeft().x)) ? bestBlue : bestRed;

        //Draw the box surrounding both contours
        //Get the width of the contours
        double widthBeacon = rightMostContour.right() - leftMostContour.left();

        //Center of contours is the average of centers of the contours
        Point center = new Point((leftMostContour.center().x + rightMostContour.center().x) / 2,
                (leftMostContour.center().y + rightMostContour.center().y) / 2);

        //Get the combined height of the contours
        double heightContours = Math.max(leftMostContour.bottom(), rightMostContour.bottom()) -
                Math.min(leftMostContour.top(), rightMostContour.top());

        //The largest size ratio of tested over actual is the scale ratio
        double scale = Math.max(widthBeacon / beaconActualWidth, heightContours / beaconActualHeight);

        //Define size of bounding box by scaling the actual beacon size
        Size beaconSizeFinal = new Size(beaconActualWidth * scale, beaconActualHeight * scale);

        //Get points of the bounding box
        Point beaconTopLeft = new Point(center.x - (beaconSizeFinal.width / 2),
                center.y - (beaconSizeFinal.height / 2));
        Point beaconBottomRight = new Point(center.x + (beaconSizeFinal.width / 2),
                center.y + (beaconSizeFinal.height / 2));
        Rectangle boundingBox = new Rectangle(new Rect(beaconTopLeft, beaconBottomRight));

        //Draw the rectangle containing the beacon
        Drawing.drawRectangle(img, boundingBox, new ColorRGBA(0, 255, 0), 4);

        //Tell us the height of the beacon
        //TODO later we can get the distance away from the beacon based on its height and position

        //Remove the largest index and look for the next largest
        //If the next largest is (mostly) within the area of the box, then merge it in with the largest

        //Check if the size of the largest contour(s) is about twice the size of the other
        //This would indicate one is brightly lit and the other is not

        //If this is not true, then neither part of the beacon is highly lit
        if (leftIsRed) {
            result.left = BeaconColor.RED;
            result.right = BeaconColor.BLUE;
            result.location = boundingBox;
            result.setConfidence(confidence);
            return result;
        } else {
            result.left = BeaconColor.BLUE;
            result.right = BeaconColor.RED;
            result.location = boundingBox;
            result.setConfidence(confidence);
            return result;
        }
    }

    public enum BeaconColor {
        RED,
        BLUE,
        RED_BRIGHT,
        BLUE_BRIGHT,
        UNKNOWN;

        @Override
        public String toString() {
            switch (this) {
                case RED:
                    return "red";
                case BLUE:
                    return "blue";
                case RED_BRIGHT:
                    return "RED!";
                case BLUE_BRIGHT:
                    return "BLUE!";
                case UNKNOWN:
                default:
                    return "???";
            }
        }
    }

    public static class BeaconAnalysis {

        public double imageHeight = 1;
        public double imageWidth = 1;

        //Movement values
        private Point beaconWeightedCenter;

        public Point getBeaconWeightedCenter() {
            return beaconWeightedCenter;
        }

        //Position analysis
        private double phi; //Stores are angle relative to the wall to the right of the beacon
        private double theta; //Stores are angular offset relative to when we are looking straight at the beacon

        private double radius = 0; //The distance from us to the beacon

        private boolean sameBeacon = false;

        //Physical properties in pixels
        private double pixelsWidth;
        private double pixelsHeight;

        private double buttonHeight;

        //Derivative of width of beacon in pixels with respect to theta
        private double dPixelsWdTheta;

        //If no variables are changing call this method so the system can get a sense of the variance due to noise
        public void stationaryNoiseUpdate(double currWidth, double currHeight, boolean useButton) {
            //If no position was changed, average the current values
            pixelsWidth = (pixelsWidth + currWidth)/2;
            pixelsHeight = (pixelsHeight + currHeight)/2;
            //Recalculate radius
            calculateRadius(useButton);
        }

        //Call if robot is moving
        public void nonStationaryUpdate(double currHeight, double currButtonHeight, boolean useButton) {
            pixelsHeight = currHeight;
            buttonHeight = (currButtonHeight != 0) ? currButtonHeight : buttonHeight;
            calculateRadius(useButton);
        }

        //Call this each time theta is changed to get real time update of the derivative of width
        public void updatedPixelsWdTheta(double currWidth, boolean wasdThetaPositive) {
            dPixelsWdTheta = (wasdThetaPositive) ? currWidth - pixelsWidth : pixelsWidth - currWidth;
            pixelsWidth = currWidth;
        }

        private void calculateRadius(boolean useButton) {
            double tempRadius;
            if(useButton)
                tempRadius = Constants.BEACON_BUTTON_HEIGHT/(2*Math.tan((Constants.CAMERA_VERT_VANGLE*buttonHeight)/(2*imageHeight)));
            else
                tempRadius = Constants.BEACON_HEIGHT/(2*Math.tan((Constants.CAMERA_VERT_VANGLE*pixelsHeight)/(2*imageHeight)));
            if(sameBeacon)
                tempRadius = Math.abs(tempRadius - radius)*Constants.CM_FT_SCALE < Constants.DIST_CHANGE_THRESHOLD ? tempRadius : radius;
            radius = (tempRadius * Constants.CM_FT_SCALE < Constants.MAX_DIST_FROM_BEACON) ? tempRadius : radius;
        }
        public double getRadius() {
            return radius;
        }
        public void setRadius(double radius) {
            this.radius = radius;
        }

        //Color Analysis
        private double confidence;
        private BeaconColor left;
        private BeaconColor right;
        private Rectangle location;

        //TODO Color and CONFIDENCE should make up the results

        //TODO add Size size, Point locationTopLeft, Distance distanceApprox
        public BeaconAnalysis() {
            this.left = BeaconColor.UNKNOWN;
            this.right = BeaconColor.UNKNOWN;
            this.confidence = 0.0f;
            this.location = new Rectangle();
        }

        public BeaconAnalysis(Size imageSize)
        {
            assert left != null;
            assert right != null;
            this.left = BeaconColor.UNKNOWN;
            this.right = BeaconColor.UNKNOWN;
            this.imageWidth = imageSize.width;
            this.imageHeight = imageSize.height;
            this.confidence = 0.0f;
            this.location = new Rectangle();
        }

        public BeaconAnalysis(BeaconColor left, BeaconColor right, Rectangle location, double confidence) {
            this.left = left;
            this.right = right;
            this.confidence = confidence;
            this.location = location;
        }

        public void setConfidence(double confidence) {
            this.confidence = confidence;
        }

        public Rectangle getBoundingBox() {
            return location;
        }

        public Point getTopLeft() {
            return location.topLeft();
        }

        public Point getBottomRight() {
            return location.bottomRight();
        }

        public Point getCenter() {
            return location.center();
        }

        public double getPixelWidth() {
            return location.width();
        }

        public double getPixelHeight() {
            return location.height();
        }

        public BeaconColor getStateLeft() {
            return left;
        }

        public BeaconColor getStateRight() {
            return right;
        }

        public double getConfidence() {
            return MathUtil.coerce(0, 1, confidence);
        }

        public String getConfidenceString() {
            final DecimalFormat format = new DecimalFormat("0.000");
            return format.format(getConfidence() * 100.0f) + "%";
        }

        public boolean isLeftKnown() {
            return left != BeaconColor.UNKNOWN;
        }

        public boolean isLeftBlue() {
            return (left == BeaconColor.BLUE_BRIGHT || left == BeaconColor.BLUE);
        }

        public boolean isLeftRed() {
            return (left == BeaconColor.RED_BRIGHT || left == BeaconColor.RED);
        }

        public boolean isRightKnown() {
            return right != BeaconColor.UNKNOWN;
        }

        public boolean isRightBlue() {
            return (right == BeaconColor.BLUE_BRIGHT || right == BeaconColor.BLUE);
        }

        public boolean isRightRed() {
            return (right == BeaconColor.RED_BRIGHT || right == BeaconColor.RED);
        }

        public boolean isBeaconFound() {
            return isLeftKnown() && isRightKnown();
        }

        public boolean isBeaconFullyLit() {
            return (left == BeaconColor.BLUE_BRIGHT || left == BeaconColor.RED_BRIGHT) &&
                    (right == BeaconColor.BLUE_BRIGHT || right == BeaconColor.RED_BRIGHT);
        }

        public String getColorString() {
            return left.toString() + ", " + right.toString();
        }

        public String getLocationString() {
            return getCenter().toString();
        }

        @Override
        public String toString() {
            return "Color: " + getColorString() + "\r\n Location: " + getLocationString() + "\r\n Confidence: " + getConfidenceString();
        }
    }
}