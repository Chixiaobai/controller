# -*- mode: python, encoding: utf-8 -*-
import os
import sys
import pprint
import json
import struct
import generate_base
import traceback
from string import Template


class FileHandler:
    def __init__(self, file_name, mode, encoding="utf-8"):
        self.file = open(file_name, mode)
        self.encoding = encoding

    def write(self, content):
        if isinstance(content, str):
            content = content.encode(self.encoding)
        self.file.write(content)

    def read(self, n=-1):
        data = self.file.read(n)
        return data.decode(self.encoding)

    def close(self):
        self.file.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        return self.file.__exit__(exc_type, exc_value, traceback)


def find_message_definition(name, allow_rescan=True, with_abs_name=False):
    for directory in generate_base.message_definition_dirs:
        file_path = os.path.join(directory, name)
        if os.path.isfile(file_path):
            if not with_abs_name:
                return file_path
            else:
                return file_path, name
    if os.path.isfile(name):
        if not with_abs_name:
            return os.path.realpath(name)
        else:
            return os.path.realpath(name), os.path.realpath(name)

    if allow_rescan:
        generate_base.rescan_message_definition_dirs()
        return find_message_definition(
            name, allow_rescan=False, with_abs_name=with_abs_name
        )
    paths = "\n".join(generate_base.message_definition_dirs)
    raise Exception(
        f"Message definition of name {name!r} not found in these directories:{paths}\n"
    )


def get_message_definition_from_filename(file_name):
    best_directory = None
    for directory in generate_base.message_definition_dirs:
        if file_name.startswith(directory) and (
            best_directory is None or len(directory) > len(best_directory)
        ):
            best_directory = directory
    if best_directory is None:
        raise Exception(f"Could not get message_definition name for path {file_name!r}")
    if best_directory[-1] != "/":
        best_directory += "/"
    return file_name[len(best_directory) :]


class DataType:
    def __init__(self, name, pyformat, aliases=None):
        self.name = name
        self.pyformat = pyformat
        self.size = struct.calcsize(self.pyformat)
        self.aliases = aliases if aliases is not None else []
        nptype_name = (
            name[:-2]
            if name.endswith("_t")
            else self.aliases[0][:-2]
            if self.aliases
            else name
        )

    def get_c_name(self):
        return self.name

    def get_sizeof(self):
        return self.size


data_types = [
    DataType("float", "f", aliases=["float32_t"]),
    DataType("double", "d", aliases=["float64_t"]),
    DataType("char", "b",aliases=["int8_t"]),
    DataType("uint8_t", "B"),
    DataType("int16_t", "h", aliases=["short"]),
    DataType("uint16_t", "H"),
    DataType("int32_t", "i", aliases=["int"]),
    DataType("uint32_t", "I"),
    DataType("int64_t", "q", aliases=["long long"]),
    DataType("uint64_t", "Q"),
    DataType("bool", "?"),
]
data_type_map = {}
for t in data_types:
    data_type_map[t.name] = t
    for alias in t.aliases:
        data_type_map[alias] = t


class MessageDefinition:
    """
    this class represents a single message definition
    """

    def __init__(self, name, file_name=None, text=None, encode="utf-8"):
        self.name = name
        self.file_name = file_name
        self.encode = encode  # do first encode text if it is a unicode!
        generate_base.logger.debug(
            f"loading message definition {name!r} from {file_name!r}"
        )
        if file_name:
            self.load_from_file(file_name)
        elif text is not None:
            self.load(text)
        else:
            self._reset()

    def _reset(self):
        self.is_service = False
        self.is_dynamic = False
        self.fields = []
        self.resp_fields = []  # only used if self.is_service == True
        self.defines = []
        self.define_abs_names = {}
        self.filename = ""

    def load_from_file(self, file_name):
        with FileHandler(file_name, "rb") as file_pointer:
            text = file_pointer.read()
        self.load(text, file_name)

    def load(self, text: str, file_name: str = "<text>") -> None:
        """
        Load the message definition from the given text.

        Args:
            text: The message definition text.
            file_name: The name of the file. Default is "<text>".

        Returns:
            None
        """
        self._reset()
        self.filename = file_name
        defines_set = set()
        service_mode = "req"
        line_no = 0
        first_non_empty = True
        pointers = dict(req=[], resp=[])
        for line in text.split("\n"):
            line_no += 1
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if "#" in line:
                line = line.split("#", 1)[0].strip()
            if first_non_empty:
                first_non_empty = False
                if line == "service":
                    self.is_service = True
                    continue
            if self.is_service:
                if line == "request":
                    service_mode = "req"
                    continue
                if line == "response":
                    service_mode = "resp"
                    continue
            self._parse_line_content(
                line, file_name, line_no, defines_set, service_mode, pointers
            )
        if self.is_service or self.is_dynamic:
            #处理req和resp指针
            todo = (("req", self.fields),)
            for mode, fields in todo:
                for ptr in pointers[mode]:
                    for i, (ft, file_name, fc) in enumerate(fields):
                        if file_name == ptr:
                            break
                    else:
                        raise Exception("alg error")
                    ptr_len = "%s_len" % ptr
                    for k, (ft, file_name, fc) in enumerate(fields):
                        if file_name == ptr_len:
                            break
                    else:
                        fields.insert(i, ["uint32_t", ptr_len, 1])
                        continue
                    if k == i - 1:
                        continue
                    field = fields[k]
                    del fields[k]
                    fields.insert(i, field)
            
            todo = (("resp", self.resp_fields),)
            for mode, fields in todo:
                for ptr in pointers[mode]:
                    for i, (ft, file_name, fc) in enumerate(fields):
                        if file_name == ptr:
                            break
                    else:
                        raise Exception("alg error")
                    ptr_len = "%s_len" % ptr
                    for k, (ft, file_name, fc) in enumerate(fields):
                        if file_name == ptr_len:
                            break
                    else:
                        fields.insert(i, ["uint32_t", ptr_len, 1])
                        continue
                    if k == i - 1:
                        continue
                    field = fields[k]
                    del fields[k]
            

    def _parse_line_content(
        self, line, file_name, line_no, defines_set, service_mode, pointers
    ):
        try:
            first_word, rest = line.split(" ", 1)
        except ValueError:
            raise SyntaxError(
                f"syntax error in message definition file\n{file_name}:{line_no}: unknown line syntax {line}"
            )

        if first_word == "import":
            self._handle_import_statement(rest, file_name, line_no, defines_set)
            return
        if first_word.endswith("*"):
            self.is_dynamic = True
            is_pointer = True
            first_word = first_word[:-1]
        else:
            is_pointer = False
        # assume first_word to be data type name
        if first_word in data_type_map or first_word in defines_set:
            field = rest.strip()
            if "[" in field:
                if is_pointer:
                    raise Exception(
                        "syntax error in message definition file\n%s:%d: currently array of pointer-types like %s* are not implemented!"
                        % (file_name, line_no, first_word)
                    )
                field, count = field.split("[", 1)
                count = int(eval(count.strip(" []")))
            else:
                count = 1
            for invalid in ";.,+-*/{}()#$äöü?'`\"\\":
                if invalid in field:
                    raise Exception(
                        "syntax error in message definition file\n%s:%d: char %r is not allowed in a field name: %r!"
                        % (file_name, line_no, invalid, field)
                    )
            if is_pointer:
                first_word += "*"
                pointers[service_mode].append(field)
            field = [first_word, field, count]  # ['int', 'version_minor', 1],
            if (self.is_service) and service_mode == "resp":
                self.resp_fields.append(field)
            else:
                self.fields.append(field)
            return
        raise Exception(
            "syntax error in message definition file\n%s:%d: unknown type %r"
            % (file_name, line_no, first_word)
        )

    def _handle_import_statement(self, rest, file_name, line_no, defines_set):
        new_define, from_file = rest.strip().split("from", 1)
        new_define = new_define.strip()
        if '"' in new_define:
            raise Exception(
                "syntax error in message definition file\n%s:%d: invalid define/type-name %r in line\n%s"
                % (file_name, line_no, new_define, line)
            )
        from_file = from_file.strip(" \r\t\n")[1:-1]
        abs_name = os.path.join(os.path.dirname(self.name), from_file)
        generate_base.logger.debug(
            f"importing {new_define!r} from {from_file!r} (abs_name {abs_name!r})"
        )
        tfn = os.path.join(os.path.dirname(file_name), from_file)
        if not os.path.isfile(tfn):
            try:
                tfn, abs_name = find_message_definition(from_file, with_abs_name=True)
            except Exception:
                raise Exception(
                    "%s:%d: error: there is a type-define %r to a missing file %r!\n%s"
                    % (
                        file_name,
                        line_no,
                        new_define,
                        from_file,
                        traceback.format_exc(),
                    )
                )
        defines_set.add(new_define)
        self.defines.append([new_define, tfn, from_file])
        self.define_abs_names[new_define] = abs_name

    def calculate_size(self):
        # dependencies
        define_sizes = {}
        for alias, deffile, defname in self.defines:
            defmd = MessageDefinition(defname, deffile)
            define_sizes[alias] = defmd.calculate_size()
        fields = []
        size = 0
        for field_type, field_name, field_count in self.fields:
            if field_type.endswith("*"):
                continue
            if field_type in define_sizes:
                size += define_sizes[field_type] * field_count
            else:
                size += data_type_map[field_type].get_sizeof() * field_count
        return size

    def __str__(self):
        if self.is_service:
            return "fields:\nres:\n%s\nresp:\n%s\nsource:\n%s\nimport:\n%s" % (
                pprint.pformat(self.fields),
                pprint.pformat(self.resp_fields),
                pprint.pformat(self.file_name),
                pprint.pformat(self.defines),
            )
        else:
            return "fields:\n%s\nsource:\n%s\nimport:\n%s\n" % (
                pprint.pformat(self.fields),
                pprint.pformat(self.file_name),
                pprint.pformat(self.defines),
            )

    def  getJson(self):
        """
        获取json格式的消息定义
        """

        if(self.is_service):
            output_dict = {
                "is_service": True,
                "req": self.fields,
                "resp": self.resp_fields,
                "source":self.file_name,
                "import": []
            }
        else:
            output_dict = {
                "is_service": False,
                "fields": self.fields,
                "source": self.file_name,
                "import": [],
                "full_name":self.name,
                "calculate_size": self.calculate_size()
            }
        
        for alias, deffile, defname in self.defines:
                defmd = MessageDefinition(defname, deffile)
                substring=defmd.getJson()
                subjson=json.loads(substring)
                output_dict["import"].append({"alias":alias,"subjson":subjson})
        return json.dumps(output_dict, ensure_ascii=False)





    def generate_c_serialize(self, fields):
        """
        生成序列化相关代码
        """

        def serialize_field(c_type, field_name, field_count, serialize_code):
            if field_count > 1:
                serialize_code += f"\tfor(int i=0; i<{field_count}; i++){{\n"
                serialize_code += f"\treqStream<<{field_name}[i];\n"
                serialize_code += "\t}\n"
            else:
                serialize_code += f"\treqStream<<{field_name};\n"
            return serialize_code

        serialize_code = ""
        for i in range(len(fields)):
            field_type, field_name, field_count = fields[i]
            if field_type in data_type_map:
                # Built-in data type
                c_type_name = data_type_map[field_type].get_c_name()
                serialize_code = serialize_field(
                    c_type_name, field_name, field_count, serialize_code
                )
            else:
                # Custom type
                for alias, deffile, defname in self.defines:
                    if alias == field_type:
                        if field_count > 1:
                            serialize_code += (
                                f"\tfor(int i=0; i<{field_count}; i++){{\n"
                            )
                            serialize_code += (
                                f"\t{field_name}[i].serialize(reqStream);\n"
                            )
                            serialize_code += "\t}\n"
                        else:
                            serialize_code += f"\t{field_name}.serialize(reqStream);\n"
            # Pointer type
            if field_type.endswith("*"):
                last_field_type, last_field_name, _ = fields[i - 1]
                if last_field_name.startswith(field_name) and last_field_name.endswith(
                    "_len"
                ):
                    serialize_code += f"\tif({last_field_name}>0 && {field_name}!=nullptr){{\n"
                    serialize_code += f"\tfor(uint32_t i=0; i<{last_field_name}; i++){{\n"
                    if field_type.strip("*") in data_type_map:
                        c_type_name = data_type_map[field_type.strip("*")].get_c_name()
                        serialize_code += f"\treqStream <<{field_name}[i];\n"
                        serialize_code += "\t}\n"
                        serialize_code += "\t}\n"
                    else:
                        serialize_code += f"\t{field_name}[i].serialize(reqStream);\n"
                        serialize_code += "\t}\n"
                        serialize_code += "\t}\n"

        return serialize_code

    def generate_c_deserialize(self, fields):
        """
        生成反序列化相关代码
        """

        def deserialize_field(c_type, field_name, field_count, deserialize_code):
            if field_count > 1:
                deserialize_code += f"\tfor(int i=0; i<{field_count}; i++){{\n"
                deserialize_code += f"\trespStream>>{field_name}[i];\n"
                deserialize_code += "\t}\n"
            else:
                deserialize_code += f"\trespStream>>{field_name};\n"
            return deserialize_code

        skip_lines = 0
        deserialize_code = ""
        for i in range(len(fields)):
            if skip_lines > 0:
                skip_lines -= 1
                continue  # 跳过行
            field_type, field_name, field_count = fields[i]
            if field_type in data_type_map:
                # Built-in data type
                c_type_name = data_type_map[field_type].get_c_name()
                deserialize_code = deserialize_field(
                    c_type_name, field_name, field_count, deserialize_code
                )
            else:
                # Custom type
                for alias, deffile, defname in self.defines:
                    if alias == field_type:
                        if field_count > 1:
                            deserialize_code += (
                                f"\tfor(int i=0; i<{field_count}; i++){{\n"
                            )
                            deserialize_code += (
                                f"\t{field_name}[i].deserialize(respStream);\n"
                            )
                            deserialize_code += "\t}\n"
                        else:
                            deserialize_code += (
                                f"\t{field_name}.deserialize(respStream);\n"
                            )
            # Pointer type
            if field_type.endswith("*"):
                next_field_type, last_field_name, _ = fields[i - 1]
                if last_field_name.startswith(field_name) and last_field_name.endswith(
                    "_len"
                ):
                    deserialize_code += (
                        f"\tif({last_field_name}>0 && {field_name}==nullptr){{\n"
                    )
                    deserialize_code += f"\t\t{field_name}=new std::remove_pointer<decltype({field_name})>::type[{last_field_name}];\n"
                    deserialize_code += "\t}\n"
                    deserialize_code += f"\tfor(uint32_t i=0; i<{last_field_name}; i++){{\n"
                    if field_type.strip("*") in data_type_map:
                        c_type_name = data_type_map[field_type.strip("*")].get_c_name()
                        deserialize_code += f"\trespStream>>{field_name}[i];\n"
                        deserialize_code += "\t}\n"
                    else:
                        deserialize_code += (
                            f"\t{field_name}[i].deserialize(respStream);\n"
                        )
                        deserialize_code += "\t}\n"
        return deserialize_code

    def generate_init_and_finit_code(self, fields):
        """
        生成初始化和释放指针相关代码
        """
        init_code = ""
        finit_code = ""
        for i in range(len(fields)):
            field_type, field_name, field_count = fields[i]
            # Pointer type
            if field_type.endswith("*"):
                last_field_type, last_field_name, _ = fields[i - 1]
                if last_field_name.startswith(field_name) and last_field_name.endswith(
                    "_len"
                ):
                    init_code += f"\t{field_name} = nullptr;\n"
                    init_code += f"\t{last_field_name} = 0;\n"
                    finit_code += f"\tif({field_name}!=nullptr){{\n"
                    finit_code += f"\t\tdelete[] {field_name};\n"
                    finit_code += f"\t\t{field_name}=nullptr;\n"
                    finit_code += "\t}\n"

        return init_code, finit_code


class MessageDefinitionHeader:
    """
    this class represents a to-be-generated C/C++ header file.
    it can read in message definition files and generate C-source code
    """

    def __init__(self, service_base_map=None):
        self.message_definitions = []
        self.message_definition = None
        self.clnt = None
        self.defined_signature_names = set()
        self.service_base_map = service_base_map or {}
        self.wrapping_namespaces = []
        self.md_prefix_ns_define = "SL_MD_PREFIX_NS"
        self.service_header_template = "service_header.sl"
        self.struct_header_template = "struct_header.sl"

    def add_message_definition(self, message_definition_name):
        if isinstance(message_definition_name, MessageDefinition):
            md = message_definition_name
        else:
            file_name = find_message_definition(message_definition_name)
            md = MessageDefinition(message_definition_name, file_name)
            generate_base.logger.debug(md)
            generate_base.logger.debug("size: %s" % md.calculate_size())
            # generate_base.logger.debug("size: %s" % md.generate_c_serialize())

        md.map_base_to = self.service_base_map.get(md.name)
        self.message_definition = md
        self.message_definitions.append(md)

    def _generate_struct_name(self, n):
        return n.replace(" ", "_").replace("/", "_").replace("\\", "_") + "_t"

    def _generate_struct_name_for(self, md):
        return self._generate_struct_name(md.name)

    def generate(self, file_pointer=None, output_fn=None):
        """ """
        generate_base.logger.debug("generating for %s" % output_fn)
        if file_pointer is None:
            file_pointer = sys.stdout
        if output_fn is None:
            output_fn = "messages.sl.h"
        define_name = os.path.basename(output_fn).upper().replace(".", "_")
        file_pointer.write(
            "#ifndef %s\n#define %s\n\n#include <stdint.h>\n"
            % (define_name, define_name)
        )
        self.defined_signature_names = set()

        typedef_name = self._generate_struct_name_for(self.message_definition)
        self.generate_md(file_pointer, typedef_name, self.message_definition, True)
        file_pointer.write("\n#endif // %s\n" % define_name)

    def get_namespace(self, md_name):
        namespace = list(self.wrapping_namespaces)
        namespace.extend(md_name.split("/")[:-1])
        ons = "\n".join(["namespace %s {" % n for n in namespace])
        cns = "\n".join(["} // namespace %s" % n for n in namespace])
        return ons, cns

    def generate_md(
        self, file_pointer, typedef_name, md, top_level=False, return_signature=False
    ):
        generate_base.logger.debug(
            "generate_md typedef_name %r md.name %r, is_service %r"
            % (typedef_name, md.name, md.is_service)
        )
        # dependencies
        define_map = {}
        signature_map = {}
        define_sizes = {}
        for defname, deffile, headfile in md.defines:
            abs_name = md.define_abs_names[defname]
            if deffile == md.filename:
                # self recursion!
                defmd = md
                defsn = typedef_name
                define_map[defname] = defsn
            else:
                defmd = MessageDefinition(abs_name, deffile)
                defmd.map_base_to = self.service_base_map.get(defmd.name)
                defsn = self._generate_struct_name(defmd.name)
                define_map[defname] = defsn
                if top_level:
                    file_pointer.write("#include \"" + headfile + ".sl.h\"\n")
                define_sizes[defname], signature_map[defname] = self.generate_md(
                    file_pointer, defsn, defmd, top_level=False, return_signature=True
                )

        struct_name = typedef_name[:-2]

        def generate_field_signature(
            field_type, field_name, field_count, define_map, signature_map, define_sizes
        ):
            """
            生成字段的签名
            """
            if field_type.endswith("*"):
                ptr = "*"
                field_type = field_type[:-1]
            else:
                ptr = ""
            if field_type in define_map:
                c_type_name = define_map[field_type]
                if c_type_name == typedef_name:
                    if not ptr:
                        raise Exception(
                            "self-recursion detected in md file\n%s\nfor field %s which is not a pointer!"
                            % (md.filename, field_name)
                        )
                    sig = "[]"  # not yet known!
                    s = 0  # not really needed
                else:
                    sig = "[%s]" % (signature_map[field_type])
                    s = define_sizes[field_type]
            else:
                c_type_name = data_type_map[field_type].get_c_name()
                sig = c_type_name
                s = data_type_map[field_type].get_sizeof()
            return sig, s, ptr, c_type_name

        def generate_field_declaration(
            field_type, field_name, field_count, add_indent, ptr, c_type_name
        ):
            """
            生成字段声明
            """
            if field_count > 1:
                field_decl = add_indent + "\t%s %s[%d];" % (
                    c_type_name,
                    field_name,
                    field_count,
                )
            else:
                field_decl = add_indent + "\t%s%s %s;" % (c_type_name, ptr, field_name)
            return field_decl

        def append_field_definitions(field_defs, add_indent=""):
            """
            生成 C 代码和签名
            """
            fields = []
            signature = []
            size = 0
            need_self_forward_declaration = False
            for field_type, field_name, field_count in field_defs:
                sig, s, ptr, c_type_name = generate_field_signature(
                    field_type,
                    field_name,
                    field_count,
                    define_map,
                    signature_map,
                    define_sizes,
                )
                if not ptr:
                    size += s * field_count
                field_decl = generate_field_declaration(
                    field_type, field_name, field_count, add_indent, ptr, c_type_name
                )
                fields.append(field_decl)
                signature.append("%s%s %s %s" % (sig, ptr, s, field_count))
            return fields, signature, size, need_self_forward_declaration

        def generate_c_header_packet(md):
            (
                fields,
                signature,
                size,
                need_self_forward_declaration,
            ) = append_field_definitions(md.fields)
            if file_pointer and typedef_name not in self.defined_signature_names:
                self.defined_signature_names.add(typedef_name)
                (init_code, finit_code) = md.generate_init_and_finit_code(md.fields)
                template_data = dict(
                    typedef_name=typedef_name,
                    struct_name=struct_name,
                    fields="\n".join(fields),
                    serialize_code=md.generate_c_serialize(md.fields),
                    deserialization_code=md.generate_c_deserialize(md.fields),
                    init_code=init_code,
                    finit_code=finit_code,
                )
                file_pointer.write(
                    fill_template(self.struct_header_template, template_data)
                )

            return size, ",".join(signature)

        def generate_c_header_service(md):
            fields = []
            typedefs = []
            if md.fields:
                add_fields, signature, s, nsf_decl = append_field_definitions(
                    md.fields, add_indent="\t"
                )
                fields.append("\tstruct  %s_request_t  : public Message {" % struct_name)
                fields.extend(add_fields)
                fields.append("\t%s_request_t(){\n${req_init_code}}" % struct_name)
                fields.append("\t~%s_request_t(){\n${req_finit_code}}" % struct_name)
                fields.append(
                    "\tvoid serialize(CAbstractStream & reqStream) const override {\n${req_serialize_code}\n}"
                )
                fields.append(
                    "\tvoid deserialize(CAbstractStream & respStream) override {\n${req_deserialization_code}\n}"
                )
                # fields.append(
                #     "\tvoid serializeToArray(void*& data, int& size) const {\n"
                # )
                # fields.append("\t\t CAbstractStream  reqStream;\n")
                # fields.append("\t\t serialize(reqStream);;\n")
                # fields.append("\t\t int len=reqStream.getLength();\n")
                # fields.append("\t\t void *temp=malloc(len);\n")
                # fields.append("\t\t reqStream.readObject(temp,len);\n")
                # fields.append("\t\t data=temp;\n")
                # fields.append("\t\t size=len;\n")
                # fields.append("\t}\n")

                # fields.append(
                #     "\tvoid deserializeFromArray(const void* data, int size){\n"
                # )
                # fields.append("\t\t CAbstractStream  respStream;\n")
                # fields.append("\t\t respStream.writeObject(data,size);\n")
                # fields.append("\t\t deserialize(respStream);\n")
                # fields.append("\t}\n")

                fields.append("\t} req;")
                typedefs.append((struct_name + "_request_t", "request_t"))

                req_signature = ",".join(signature)
                service_signature = "%s|" % (req_signature)
            else:
                fields.append("\tstruct  %s_request_t  : public Message {" % struct_name)
                fields.append("\t%s_request_t(){\n${req_init_code}}" % struct_name)
                fields.append("\t~%s_request_t(){\n${req_finit_code}}" % struct_name)
                fields.append(
                    "\tvoid serialize(CAbstractStream & reqStream) const override {}"
                )
                fields.append(
                    "\tvoid deserialize(CAbstractStream & respStream) override {}"
                )
                fields.append("\t} req;")
                typedefs.append((struct_name + "_request_t", "request_t"))

                req_signature =None
                service_signature =None
            if md.resp_fields:
                add_fields, signature, s, nsf_decl = append_field_definitions(
                    md.resp_fields, add_indent="\t"
                )

                fields.append("\tstruct  %s_response_t : public Message {" % struct_name)
                fields.extend(add_fields)
                fields.append("\t%s_response_t(){\n${resp_init_code}}" % struct_name)
                fields.append("\t~%s_response_t(){\n${resp_finit_code}}" % struct_name)
                fields.append(
                    "\tvoid serialize(CAbstractStream & reqStream) const override {\n${resp_serialize_code}\n}"
                )
                fields.append(
                    "\tvoid deserialize(CAbstractStream & respStream) override {\n${resp_deserialization_code}\n}"
                )

                fields.append("\t} resp;")
                typedefs.append((struct_name + "_response_t", "response_t"))
            else:
                fields.append("\tstruct  %s_response_t : public Message {" % struct_name)
                fields.append("\t%s_response_t(){\n${resp_init_code}}" % struct_name)
                fields.append("\t~%s_response_t(){\n${resp_finit_code}}" % struct_name)
                fields.append(
                    "\tvoid serialize(CAbstractStream & reqStream) const override {}"
                )
                fields.append(
                    "\tvoid deserialize(CAbstractStream & respStream) override {}"
                )

                fields.append("\t} resp;")
                typedefs.append((struct_name + "_response_t", "response_t"))

            if file_pointer and typedef_name not in self.defined_signature_names:
                self.defined_signature_names.add(typedef_name)
                bn = os.path.basename(md.name)
                name = md.map_base_to or bn
                svc_data_name = "data"
                class_name = name + "_base"
                if md.resp_fields:
                    clear_resp = (
                        "\n\t\tmemset(&"
                        + svc_data_name
                        + ".resp, 0, sizeof("
                        + svc_data_name
                        + ".resp));"
                    )
                else:
                    clear_resp = ""
                ns_opening, ns_closing = self.get_namespace(md.name)
                if typedefs:
                    typedefs = (
                        "\n"
                        + "#ifdef __cplusplus\n"
                        + "\n".join(
                            ["\ttypedef %s %s;" % (s, td) for s, td in typedefs]
                        )
                        + "\n#endif\n"
                    )
                else:
                    typedefs = ""
                template_data = dict(
                    md_prefix_ns_define=self.md_prefix_ns_define,
                    ns_opening=ns_opening,
                    ns_closing=ns_closing,
                    class_name=class_name,
                    full_name=md.name.replace("/", "_").replace("\\", "_"),
                    name=name,
                    typedef_name=typedef_name,
                    struct_name=struct_name,
                    mdname=md.name,
                    clear_resp=clear_resp,
                    fields="\n".join(fields),
                    service_signature=service_signature,
                    typedefs=typedefs,
                )
                template = Template(
                    fill_template(self.service_header_template, template_data)
                )
                (req_init_code, req_finit_code) = md.generate_init_and_finit_code(
                    md.fields
                )
                (resp_init_code, resp_finit_code) = md.generate_init_and_finit_code(
                    md.resp_fields
                )
                template_data = dict(
                    req_serialize_code=md.generate_c_serialize(md.fields),
                    req_deserialization_code=md.generate_c_deserialize(md.fields),
                    req_init_code=req_init_code,
                    req_finit_code=req_finit_code,
                    resp_serialize_code=md.generate_c_serialize(md.resp_fields),
                    resp_deserialization_code=md.generate_c_deserialize(md.resp_fields),
                    resp_init_code=resp_init_code,
                    resp_finit_code=resp_finit_code,
                )

                file_pointer.write(template.safe_substitute(template_data))
            # return 0 here, size has no real meaning for a service
            return 0, service_signature

        if md.is_service:
            size, signature = generate_c_header_service(md)
        elif top_level:  # recursion
            size, signature = generate_c_header_packet(md)
        else:
            size = 0
            signature = ""
        if return_signature:
            return size, signature
        return size

    def find_md_for_filename(self, filename):
        for md in self.message_definitions:
            if md.filename == filename:
                return md
        return None


def compute_message_definition_size(name):
    file_name = find_message_definition(name)
    md = MessageDefinition(name, file_name)
    return md.calculate_size()


def compute_message_definition_string(name):
    def calculate_string(md):
        # dependencies
        txt = ""

        for field_type, field_name, field_count in md.fields:
            typ = data_type_map[field_type].pyformat
            txt = txt + typ + "|" + str(field_name) + "|" + str(field_count) + "||"

        return txt

    file_name = find_message_definition(name)
    md = MessageDefinition(name, file_name)
    return calculate_string(md)


def fill_template(template_fn, data):
    file_name = os.path.join(os.path.dirname(__file__), "templates", template_fn)
    with FileHandler(file_name, "rb") as file_pointer:
        template = Template(file_pointer.read())
    return template.safe_substitute(data)
