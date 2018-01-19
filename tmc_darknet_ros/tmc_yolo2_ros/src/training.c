#include <tmc_darknet/box.h>
#include <tmc_darknet/cost_layer.h>
#include <tmc_darknet/network.h>
#include <tmc_darknet/parser.h>
#include <tmc_darknet/region_layer.h>
#include <tmc_darknet/utils.h>


#include <tmc_darknet/option_list.h>

#ifdef OPENCV
#include <opencv2/highgui/highgui_c.h>
#endif

// darknet/detector.cの実装から、学習に用いる箇所のみ抽出
void training(char *datacfg, char *cfgfile, char *weightfile, int *gpus, int ngpus, int clear) {
    list *options = read_data_cfg(datacfg);
    char *train_images = option_find_str(options, "train", "data/train.list");
    char *backup_directory = option_find_str(options, "backup", "/backup/");

    srand(time(0));
    char *base = basecfg(cfgfile);
    printf("%s\n", base);
    float avg_loss = -1;
    network *nets = calloc(ngpus, sizeof(network));

    srand(time(0));
    int seed = rand();
    int i;
    for (i = 0; i < ngpus; ++i) {
        srand(seed);
#ifdef GPU
        cuda_set_device(gpus[i]);
#endif
        nets[i] = parse_network_cfg(cfgfile);
        if (weightfile) {
            load_weights(&nets[i], weightfile);
        }
        if (clear) {
          *nets[i].seen = 0;
        }
        nets[i].learning_rate *= ngpus;
    }
    srand(time(0));
    network net = nets[0];

    int imgs = net.batch * net.subdivisions * ngpus;
    printf("Learning Rate: %g, Momentum: %g, Decay: %g\n",
           net.learning_rate, net.momentum, net.decay);
    data train, buffer;

    layer l = net.layers[net.n - 1];

    int classes = l.classes;
    float jitter = l.jitter;

    list *plist = get_paths(train_images);
    char **paths = (char **)list_to_array(plist);

    load_args args = {0};
    args.w = net.w;
    args.h = net.h;
    args.paths = paths;
    args.n = imgs;
    args.m = plist->size;
    args.classes = classes;
    args.jitter = jitter;
    args.num_boxes = l.max_boxes;
    args.d = &buffer;
    args.type = DETECTION_DATA;
    args.threads = 8;

    args.angle = net.angle;
    args.exposure = net.exposure;
    args.saturation = net.saturation;
    args.hue = net.hue;

    pthread_t load_thread = load_data(args);
    clock_t time;
    int count = 0;
    while (get_current_batch(net) < net.max_batches) {
        if (l.random && count++%10 == 0) {
            printf("Resizing\n");
            int dim = (rand() % 10 + 10) * 32;
            if (get_current_batch(net)+200 > net.max_batches) dim = 608;
            printf("%d\n", dim);
            args.w = dim;
            args.h = dim;

            pthread_join(load_thread, 0);
            train = buffer;
            free_data(train);
            load_thread = load_data(args);

            for (i = 0; i < ngpus; ++i) {
                resize_network(nets + i, dim, dim);
            }
            net = nets[0];
        }
        time = clock();
        pthread_join(load_thread, 0);
        train = buffer;
        load_thread = load_data(args);

        printf("Loaded: %lf seconds\n", sec(clock()-time));

        time = clock();
        float loss = 0;
#ifdef GPU
        if (ngpus == 1) {
            loss = train_network(net, train);
        } else {
            loss = train_networks(nets, ngpus, train, 4);
        }
#else
        loss = train_network(net, train);
#endif
        if (avg_loss < 0) avg_loss = loss;
        avg_loss = avg_loss*.9 + loss*.1;

        i = get_current_batch(net);
        printf("%d: %f, %f avg, %f rate, %lf seconds, %d images\n",
               get_current_batch(net), loss, avg_loss, get_current_rate(net),
               sec(clock()-time), i*imgs);
        if (i%1000 == 0 || (i < 1000 && i%100 == 0)) {
#ifdef GPU
          if (ngpus != 1) {
            sync_nets(nets, ngpus, 0);
          }
#endif
            char buff[256];
            sprintf(buff, "%s/%s_%d.weights", backup_directory, base, i);
            save_weights(net, buff);
        }
        free_data(train);
    }
#ifdef GPU
    if (ngpus != 1) {
      sync_nets(nets, ngpus, 0);
    }
#endif
    char buff[256];
    sprintf(buff, "%s/%s_final.weights", backup_directory, base);
    save_weights(net, buff);
}

int main(int argc, char **argv) {
  gpu_index = find_int_arg(argc, argv, "-i", 0);
  if (find_arg(argc, argv, "-nogpu")) {
    gpu_index = -1;
  }
#ifndef GPU
  gpu_index = -1;
#else
  if (gpu_index >= 0) {
    cuda_set_device(gpu_index);
  }
#endif
  if (argc < 4) {
    fprintf(stderr, "usage: rosrun tmc_darknet_ros training [data] [cfg] [weights]\n");
    return;
  }
  char *gpu_list = find_char_arg(argc, argv, "-gpus", 0);
  int *gpus = 0;
  int gpu = 0;
  int ngpus = 0;
  if (gpu_list) {
    printf("%s\n", gpu_list);
    int len = strlen(gpu_list);
    ngpus = 1;
    int i;
    for (i = 0; i < len; ++i) {
      if (gpu_list[i] == ',') ++ngpus;
    }
    gpus = calloc(ngpus, sizeof(int));
    for (i = 0; i < ngpus; ++i) {
      gpus[i] = atoi(gpu_list);
      gpu_list = strchr(gpu_list, ',')+1;
    }
  } else {
    gpu = gpu_index;
    gpus = &gpu;
    ngpus = 1;
  }
  int clear = find_arg(argc, argv, "-clear");

  char *datacfg = argv[1];
  char *cfg = argv[2];
  char *weights = argv[3];
  training(datacfg, cfg, weights, gpus, ngpus, clear);
  return 0;
}
