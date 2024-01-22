from ultralytics.data import build_dataloader
from ultralytics.data import YOLODataset
import numpy as np
import imagecorruptions
class FinalDataLoaderCorrupted(YOLODataset):

    def __init__(self, data_set, img_path, batch=1, shuffle=True, corr_type = ['frost', 'fog']):

        import math

        self.image_path = img_path
        self.batch = batch
        self.shuffle = shuffle

        self.dataset = data_set

        self.all_images = np.array(list(range(len(self.dataset))))
        np.random.shuffle(self.all_images)
        self.corruption_id = np.random.choice(len(self.dataset), size = math.ceil(0.5 *len(self.dataset)), replace=False)
        self.first_corruption = self.corruption_id[:math.ceil(0.75*len(self.corruption_id))]
        self.second_corruption = self.corruption_id[math.ceil(0.25*len(self.corruption_id)):]
        self.both_corruptions = list(set(self.first_corruption) & set(self.second_corruption))
        self.only_first = list(set(self.first_corruption) - set(self.both_corruptions))
        self.only_second = list(set(self.second_corruption) - set(self.both_corruptions))
        self.corr_type = corr_type

    def __len__(self):
        return len(self.dataset)

    def __getitem__(self, idx):

        from itertools import islice, cycle

        img_selected = self.all_images[idx]

        # Note: the line below gets the exact index from the dataset.
        # But the computation time increases 100 times.
        # Hence it is changed to get next item,
        # drawback: randomisation is lost
        #ite = next(islice(cycle(self.dataset), img_selected, None))
        ite = next((cycle(self.dataset)))
        if ite is None:
            raise IndexError
        corr = []
        if img_selected in self.only_first:
            corr = [0]
        elif img_selected in self.only_second:
            corr = [1]
        elif img_selected in self.both_corruptions:
            corr = [0, 1]

        # Apply corruptions if needed
        img = ite['img'].squeeze()
        ite['ratio_pad'] = [ite['ratio_pad']]
        ite['im_file'] = [ite['im_file']]
        ite['ori_shape'] = [list(ite['ori_shape'])]
        ite['resized_shape'] = [list(ite['resized_shape'])]
        if corr != []:
            img = (np.asarray(img.permute(1, 2, 0))*255).astype(np.uint8)
            for i in corr:
                img = imagecorruptions.corrupt(img, corruption_name=self.corr_type[i], severity=5)
            img = torch.tensor(img/255).permute(2, 0, 1).float()
        ite['img'] = img.unsqueeze(0)
        return ite