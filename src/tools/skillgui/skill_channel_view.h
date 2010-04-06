
/***************************************************************************
 *  skill_channel_view.h - TODO: Add Description
 *
 *  Created: Thu Apr 01 17:40:55 2010
 *  Copyright  2010  Patrick Podbregar [www.podbregar.com]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __GUI_UTILS_SKILL_CHANNEL_VIEW_H_
#define __GUI_UTILS_SKILL_CHANNEL_VIEW_H_

#include <gui_utils/connection_dispatcher.h>
#include <interfaces/SkillerInterface.h>

#include <string>
#include <vector>

namespace fawkes {
  class BlackBoard;
  class InterfaceDispatcher;
}

#include <gtkmm.h>
#ifdef HAVE_GLADEMM
#  include <libglademm/xml.h>
#endif
#ifdef HAVE_GCONFMM
#  include <gconfmm.h>
#endif


class SkillChannelView : public Gtk::TreeView
{

public:
  SkillChannelView();
#ifdef HAVE_GLADEMM
  SkillChannelView(BaseObjectType* cobject, const Glib::RefPtr<Gnome::Glade::Xml> ref_xml);
#endif
  virtual ~SkillChannelView();

  void set_gconf_prefix(Glib::ustring gconf_prefix);
  void setup_channels(fawkes::SkillerInterface *skiller_if);
  void update_channels();
  void clear_channels();

private:
  class SkillChannelRecord : public Gtk::TreeModelColumnRecord
  {
  public:
    SkillChannelRecord()
    {
      add(channel_number);
      add(skill_string);
      add(status);
    }

    Gtk::TreeModelColumn<unsigned> channel_number;
    Gtk::TreeModelColumn<Glib::ustring> skill_string;
    Gtk::TreeModelColumn<Glib::ustring> status;

#ifdef HAVE_GCONFMM
  Glib::RefPtr<Gnome::Conf::Client> __gconf;
#endif
  };

  class SkillString
  {
  public:
    SkillString(const char *skill_string)
    {
      parse_skill_string(std::string(skill_string));
    }

    SkillString(std::string skill_string)
    {
      parse_skill_string(skill_string);
    }

    std::string get_channel(unsigned channel_number)
    {
      std::string channel_string = "";
      if (channel_number < channel_strings.size())
	channel_string = channel_strings.at(channel_number);
      return channel_string;
    }

  private:
    void parse_skill_string(std::string skill_string)
    {
      size_t pos1 = 0;
      size_t pos2;
      while ((pos2 = skill_string.find("||", pos1)) != skill_string.npos)
      {
        channel_strings.push_back(skill_string.substr(pos1, pos2-pos1));
        pos1 = pos2 + 2;
      }
    }

    std::vector<std::string> channel_strings;

  };


  void ctor();
  std::string get_status_text(fawkes::SkillerInterface::SkillStatusEnum status);

  SkillChannelRecord skill_channel_record;
  Glib::RefPtr<Gtk::ListStore> skill_channel_list;
  SkillString *skill_string;

  fawkes::SkillerInterface *__skiller_if;
};

#endif
