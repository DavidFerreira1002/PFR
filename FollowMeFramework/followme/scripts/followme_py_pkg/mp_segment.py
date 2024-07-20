import cv2
import mediapipe as mp
import numpy as np
import matplotlib.pyplot as plt

def is_valid_contour(image_BGR, mask):
        #change from bgr to rgb
        image_RGB = cv2.cvtColor(image_BGR, cv2.COLOR_BGR2RGB)
        rgb_new = image_RGB.copy()
        #use the mask on the rgb
        for i in range(3):
                rgb_new[:, :, i] = np.multiply(image_BGR[:, :, i], mask[ :, :])
        #change it to gray
        gray_cluster = cv2.cvtColor(rgb_new, cv2.COLOR_BGR2GRAY)
        ret, tresh = cv2.threshold(gray_cluster, 127, 255, 0)
        #calculate contour
        cnt, _ = cv2.findContours(tresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        if(len(cnt) > 0):
            return True
        else:
            return False

class MediapipeInstanceSegmentation:
    #mediapipe instance segmentation at home

    def __init__(self, model_path, score_treshold = 0.5, display = False) -> None:
        #score threshold that the object identificator uses
        self.score_treshold = score_treshold
        self.display = display
        
        #instance the selfie segmentation model that is used
        #after getting the boxes of the persons to get their mask
        self.selfie_segmentation = mp.solutions.selfie_segmentation.SelfieSegmentation(model_selection=1)
        
        #instancing the object identificator model
        BaseOptions = mp.tasks.BaseOptions
        ObjectDetector = mp.tasks.vision.ObjectDetector
        ObjectDetectorOptions = mp.tasks.vision.ObjectDetectorOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        options = ObjectDetectorOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            max_results = 10,
            running_mode = VisionRunningMode.IMAGE,
            category_allowlist = "person"
        )

        self.object_identification = ObjectDetector.create_from_options(options)

    def close(self):
        #to close the models
        self.object_identification.close()
        self.selfie_segmentation.close()

    def format_detection_results(self,detection_list, masks_array, bgr):
        """
        Format detection results into a dictionary with numpy arrays for masks, boxes, scores, and classes.
        Drops persons without a mask.

        Args:
        - detection_list (list): List of detection instances, where each instance is a dictionary.
        - masks_array (numpy.ndarray): NumPy array of masks with shape (N, H, W).
        - bgr (cv2.BGR image): Used in checking if the persons mask is good.

        Returns:
        - formatted_dict (dict): Dictionary with keys:
            {"class":{
                    "scores":[fromthisclass_score_1, fromthisclass_score_2, ...]
                    "boxes":[fromthisclass_box_1, fromthisclass_box_2, ...]
                    "masks":[fromthisclass_mask_1, fromthisclass_mask_2, ...]     
                    
             "id":[-1]
                     }       
            }
            

            - 'masks' (numpy.ndarray): Array of masks with shape (N, H, W).
            - 'boxes' (numpy.ndarray): Array of bounding boxes with shape (N, 4).
            - 'scores' (numpy.ndarray): Array of detection scores with shape (N,).
            - 'class' (numpy.ndarray): Array of class labels with shape (N,).
        """

        # Initialize arrays to store results
        boxes_all = []
        scores = []
        masks_all = []

        # Iterate through each detection instance in the list
        for i, instance in enumerate(detection_list):
            # Extract bounding box information
            bounding_box = instance.bounding_box
            origin_x = bounding_box.origin_x
            origin_y = bounding_box.origin_y
            width = bounding_box.width
            height = bounding_box.height
            
            # Calculate box coordinates (xmin, ymin, xmax, ymax)
            xmin = origin_x
            ymin = origin_y
            xmax = origin_x + width
            ymax = origin_y + height
            
            # Store box coordinates
            boxes = np.array([xmin, ymin, xmax, ymax])
            
            # Extract class and score
            class_name = instance.categories[0].category_name #It needs to be "person"
            score = instance.categories[0].score

            mask = masks_array[i - 1]
            #Check if person has a mask and if it has contour, skip if it does not
            if (len(np.nonzero(mask))>0 and is_valid_contour(bgr, mask)):
                scores.append(score)
                boxes_all.append(boxes)
                masks_all.append(mask.astype(np.float32))
            #else:
                #print("Discarded person without mask")
                

        formatted_dict = {}
        if(len(detection_list) > 0 and len(scores)>0 and len(boxes)>0 and len(masks_all)>0):
            # Create formatted dictionary
            
            formatted_dict[class_name] = {}
            formatted_dict[class_name]['scores'] = []
            formatted_dict[class_name]['boxes'] = []
            formatted_dict[class_name]['masks'] = []
            formatted_dict[class_name]['id'] = []
            formatted_dict[class_name]['scores'] = scores
            formatted_dict[class_name]['boxes'] = boxes_all
            formatted_dict[class_name]['masks'] = masks_all
            formatted_dict[class_name]['id'].append(-1)# the id is set to -1 here, 
                                                #to mimic YolactInference from AiUtils, 
                                                #if you need ids feel free to add on to this, 
                                                # for now it doenst matter, its just for structure.
        
        return formatted_dict

    def get_persons_boxes_scores(self, image_BGR):
        image_RGB = cv2.cvtColor(image_BGR, cv2.COLOR_BGR2RGB)
        #change from nparray to mp.Image
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image_RGB)
        #it needs mp.Image to work
        results = self.object_identification.detect(mp_image)
        # filter the results based on he score threshold (default 0.5) and if they are a person
        filtered_results = [
            detection for detection in results.detections
            if detection.categories[0].score >= self.score_treshold
            and detection.categories[0].category_name.lower()=="person"
        ]
        return filtered_results

    def get_persons_masks(self, image_BGR):
        # Convert the BGR image to RGB.
        image_RGB = cv2.cvtColor(image_BGR, cv2.COLOR_BGR2RGB)
        
        # Process the image and get the segmentation mask.
        results = self.selfie_segmentation.process(image_RGB)
        # Generate the mask where the persons are located.
        full_mask_bool = results.segmentation_mask > self.score_treshold
        # Transform it from a np array of boolean, to a np array of 0 to 255
        full_mask = np.stack((full_mask_bool,), axis=-1).astype(np.uint8) * 255
        if self.display:
            # Convert the mask to a 3-channel image to show
            full_mask_3ch = np.stack((full_mask_bool,) * 3, axis=-1).astype(np.uint8) * 255
            
            # Create a white background image.
            bg_image = np.zeros(image_BGR.shape, dtype=np.uint8)
            bg_image[:] = (255, 255, 255)
            # # Combine the frame and the background image using the mask.
            output_image = np.where(full_mask_3ch, image_BGR, bg_image)
            combined_image = np.hstack((image_BGR, full_mask_3ch, output_image))
            return combined_image, full_mask
        else:
            return  full_mask

    def get_individual_masks_with_boxes(self, full_mask, detections):

        
        h, w, _ = full_mask.shape
        n = len(detections)
        #the output masks need to be in (N,H,W)
        #N : Number of detected objects
        #H : Height of the image
        #W : Width of the image
        out_masks = np.zeros((n,h,w), dtype=np.uint8)

        #iterate over the detected objects and retrieve the mask
        #for that, get the bounding box and create a mask with it
        #then use that mask on the full mask 
        #that should give the individual mask
        for i, detection in enumerate(detections, start=1):
            mask = np.zeros(full_mask.shape[:], dtype=np.uint8)
            bounding_box = detection.bounding_box
            
            xmin = bounding_box.origin_x
            ymin = bounding_box.origin_y
            xmax = xmin + bounding_box.width
            ymax = ymin + bounding_box.height
            left, right, top, bottom = int(xmin), int(xmax), int(ymin), int(ymax)
            mask[top:bottom, left:right] = 255

            result = cv2.bitwise_and(full_mask, full_mask, mask=mask)
            out_masks[i-1,:,:] = result
        return out_masks

    def run(self, image_BGR):
        boxes_scores = self.get_persons_boxes_scores(image_BGR)

        if self.display:
            view_full_mask, full_mask = self.get_persons_masks(image_BGR)
        else:
            full_mask = self.get_persons_masks(image_BGR)

        out_masks = self.get_individual_masks_with_boxes(full_mask, boxes_scores)
        
        if self.display:
            #while (not (cv2.waitKey(5) & 0xFF == ord('q'))):
            #    cv2.imshow('Original | Mask | Output', view_full_mask)
                # Exit the loop when 'q' is pressed.
                
            # Create a figure to display all masks together
            plt.figure(figsize=(10, 6))  # Adjust figure size as needed

            # Determine the number of rows and columns for subplot grid
            num_masks = out_masks.shape[0]
            rows = int(np.ceil(np.sqrt(num_masks)))  # Square-ish grid
            if(rows > 0):
                cols = int(np.ceil(num_masks / rows))

                # Plot each mask in a subplot
                for i in range(num_masks):
                    plt.subplot(rows, cols, i + 1)
                    plt.imshow(out_masks[i, :, :], cmap='gray')  # Assuming binary masks (0s and 1s)
                    plt.title(f'Mask {i+1}')  # Title with mask index (1-based)
                    plt.axis('off')

                plt.tight_layout()  # Adjust spacing between subplots
                plt.show()
        
        return self.format_detection_results(boxes_scores,out_masks,image_BGR), full_mask[:,:,0]
