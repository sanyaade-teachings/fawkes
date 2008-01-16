
/***************************************************************************
 *  firestation.h - Firestation
 *
 *  Created: Wed Oct 10 14:15:56 2007
 *  Copyright  2007  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __FIREVISION_TOOLS_IMAGE_VIEWER_H_
#define __FIREVISION_TOOLS_IMAGE_VIEWER_H_

#include <gtkmm.h>
#include <libglademm/xml.h>
#include <fvutils/base/roi.h>
#include <fvutils/color/colorspaces.h>
#include <netcomm/dns-sd/avahi_thread.h>
#include <netcomm/service_discovery/browse_handler.h>

class SharedMemoryImageBuffer;
class NetworkCamera;
class ShmImageLister;
class Writer;
class Reader;
class MirrorCalibTool;
class ColorTrainTool;

class Firestation : public Gtk::Window, public ServiceBrowseHandler
{
 public:
  Firestation(Glib::RefPtr<Gnome::Glade::Xml> ref_xml);
  virtual ~Firestation();

  Gtk::Window& get_window() const;

  // service browser handler
  void all_for_now();
  void cache_exhausted();
  void browse_failed( const char* name,
		      const char* type,
		      const char* domain );
  void service_added( const char* name,
		      const char* type,
		      const char* domain,
		      const char* host_name,
		      const struct sockaddr* addr,
		      const socklen_t addr_size,
		      uint16_t port,
		      std::list<std::string>& txt,
		      int flags );
  void service_removed( const char* name,
			const char* type,
			const char* domain );

 private:
  class ShmColumnRecord : public Gtk::TreeModel::ColumnRecord
  {
  public:
    ShmColumnRecord()
      {
	add(m_id); add(m_name);
      };
    Gtk::TreeModelColumn<int> m_id;
    Gtk::TreeModelColumn<Glib::ustring> m_name;
  };

  class FuseColumnRecord : public Gtk::TreeModel::ColumnRecord
    {
    public:
      FuseColumnRecord()
	{
	  add(m_id); add(m_name);
	  add(m_service_name); add(m_service_domain);
	  add(m_service_hostname); add(m_service_port);
	  add(m_image_id); add(m_image_width), add(m_image_height);
	  add(m_image_colorspace);
	};
      Gtk::TreeModelColumn<int> m_id;
      Gtk::TreeModelColumn<Glib::ustring> m_name;
      Gtk::TreeModelColumn<Glib::ustring> m_service_name;
      Gtk::TreeModelColumn<Glib::ustring> m_service_type;
      Gtk::TreeModelColumn<Glib::ustring> m_service_domain;
      Gtk::TreeModelColumn<Glib::ustring> m_service_hostname;
      Gtk::TreeModelColumn<unsigned short int> m_service_port;
      Gtk::TreeModelColumn<Glib::ustring> m_image_id;
      Gtk::TreeModelColumn<unsigned int> m_image_width;
      Gtk::TreeModelColumn<unsigned int> m_image_height;
      Gtk::TreeModelColumn<Glib::ustring> m_image_colorspace;
   };

  typedef enum
  {
    SRC_NONE,
    SRC_FILE,
    SRC_SHM,
    SRC_FUSE
  } ImageSource;

  typedef enum
  {
    MODE_VIEWER,
    MODE_COLOR_TRAIN,
    MODE_MIRROR_CALIB,
    MODE_MIRROR_CALIB_EVAL
  } OpMode;

  void save_image();
  void exit();
  void update_image();
  void open_file();
  void open_shm();
  void open_fuse();
  void post_open_img_src();

  bool image_click(GdkEventButton*);
  void resize_image(Gtk::Allocation& allocation);
  bool scale_image();
  void draw_image();
  void draw_segmentation_result();

  void ct_start();
  void ct_unselect();
  void ct_add();
  void ct_save_colormap();
  void ct_load_colormap();
  hint_t ct_get_fg_object();
  void ct_draw_colormaps();

  void mc_start();
  void mc_save();
  void mc_load();

  // widgets
  Gtk::Window* m_wnd_main;
  Gtk::Dialog* m_dlg_open_shm;
  Gtk::Dialog* m_dlg_open_fuse;
  Gtk::CheckButton* m_ckb_fuse_jpeg;
  Gtk::FileChooserDialog* m_fcd_open_image;
  Gtk::FileChooserDialog* m_fcd_save_image;
  Gtk::ToolButton* m_tbtn_exit;
  Gtk::ToolButton* m_tbtn_update;
  Gtk::ToolButton* m_tbtn_save;
  Gtk::ToolButton* m_tbtn_open_file;
  Gtk::ToolButton* m_tbtn_open_shm;
  Gtk::ToolButton* m_tbtn_open_fuse;
  Gtk::Image* m_img_image;
  Gtk::EventBox* m_evt_image;
  Gtk::TreeView* m_trv_shm_image_ids;
  Gtk::TreeView* m_trv_fuse_services;
  Gtk::Statusbar* m_stb_status;

  // color training widgets
  Gtk::RadioButton* m_rbt_ct_ball;
  Gtk::RadioButton* m_rbt_ct_field;
  Gtk::RadioButton* m_rbt_ct_lines;
  Gtk::HScale* m_scl_ct_threshold;
  Gtk::HScale* m_scl_ct_minprob;
  Gtk::Button* m_btn_ct_start;
  Gtk::Button* m_btn_ct_unselect;
  Gtk::Button* m_btn_ct_add;
  Gtk::Button* m_btn_ct_save_histos;
  Gtk::Button* m_btn_ct_save_colormap;
  Gtk::Button* m_btn_ct_load_colormap;
  Gtk::Window* m_wnd_ct_colormap;
  Gtk::Image* m_img_ct_segmentation;
  Gtk::Image* m_img_ct_colormap_local;
  Gtk::Image* m_img_ct_colormap_remote;
  Gtk::FileChooserDialog* m_fcd_ct_save_colormap;
  Gtk::FileChooserDialog* m_fcd_ct_load_colormap;

  // mirror calibration widgets
  Gtk::FileChooserDialog* m_fcd_mc_save;
  Gtk::FileChooserDialog* m_fcd_mc_load;
  Gtk::Button* m_btn_mc_start;
  Gtk::Button* m_btn_mc_load;
  Gtk::Button* m_btn_mc_save;
  Gtk::Entry* m_ent_mc_dist;
  Gtk::Entry* m_ent_mc_ori;
  

  ShmColumnRecord m_shm_columns;
  Glib::RefPtr<Gtk::ListStore> m_shm_list_store;

  FuseColumnRecord m_fuse_columns;
  Glib::RefPtr<Gtk::TreeStore> m_fuse_tree_store;

  SharedMemoryImageBuffer* m_shm_buffer;
  NetworkCamera* m_net_cam;
  Reader* m_img_reader;
  Writer* m_img_writer;

  ImageSource m_img_src;
  OpMode m_op_mode;

  // image buffer
  unsigned char* m_yuv_orig_buffer;
  unsigned char* m_yuv_draw_buffer;
  unsigned char* m_yuv_scaled_buffer;
  unsigned char* m_rgb_scaled_buffer;

  unsigned int m_img_width;
  unsigned int m_img_height;
  unsigned int m_scaled_img_width;
  unsigned int m_scaled_img_height;
  
  colorspace_t m_img_cs;
  size_t m_img_size;

  bool m_img_src_ready;

  bool m_enable_scaling;
  float m_scale_factor;

  ColorTrainTool* m_color_tool;
  MirrorCalibTool* m_calib_tool;

  AvahiThread* m_avahi_thread;
};

#endif /* __FIREVISION_TOOLS_IMAGE_VIEWER_H_ */